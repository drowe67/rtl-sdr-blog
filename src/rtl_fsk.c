/*
 * rtl_fsk, turns your Realtek RTL2832 based DVB dongle into a FSK receiver
 * 
 * Based on rtl_sdr
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 


#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#include "getopt/getopt.h"
#endif

#include "rtl-sdr.h"
#include "convenience/convenience.h"
#include "fsk.h"

#define DEFAULT_SAMPLE_RATE		240000
#define DEFAULT_BUF_LENGTH		(16 * 16384)
#define MINIMAL_BUF_LENGTH		512
#define MAXIMAL_BUF_LENGTH		(256 * 16384)
#define BUF_SZ                          256

static int do_exit = 0;
static uint32_t bytes_to_read = 0;
static rtlsdr_dev_t *dev = NULL;
static struct FSK *fsk;
static uint32_t out_block_size = DEFAULT_BUF_LENGTH;
static unsigned char *rawbuf;
static size_t nrawbuf = 0;
static int udp_debug = 0;
static int sockfd;
static struct sockaddr_in serveraddr;
static char hostname[256];
static int portno = 8001;
static uint32_t sample_counter;
static uint32_t samp_rate = DEFAULT_SAMPLE_RATE;

void usage(void)
{
	fprintf(stderr,
		"rtl_fsk, a FSK demodulator for RTL2832 based DVB-T receivers\n\n"
		"Usage:\t -f frequency_to_tune_to [Hz]\n"
		"\t[-s samplerate (default: %d Hz)]\n"
		"\t[-d device_index (default: 0)]\n"
		"\t[-g gain (default: 0 for auto)]\n"
		"\t[-p ppm_error (default: 0)]\n"
		"\t[-b output_block_size (default: 16 * 16384)]\n"
		"\t[-n number of samples to read (default: 0, infinite)]\n"
		"\t[-S force sync output (default: async)]\n"
		"\t[-u hostname (optional host to send debug information to on port 8001)\n"
		"\tfilename (a '-' dumps bits to stdout)\n\n", DEFAULT_SAMPLE_RATE);
	exit(1);
}

#ifdef _WIN32
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		do_exit = 1;
		rtlsdr_cancel_async(dev);
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	do_exit = 1;
	rtlsdr_cancel_async(dev);
}
#endif

static void udp_sendbuf(char buf[]) {
    int n;

    /* send the message to the server */
    n = sendto(sockfd, buf, strlen(buf), 0, (const struct sockaddr *)&serveraddr, sizeof(serveraddr));
    if (n < 0)  {
        fprintf(stderr, "ERROR in sendto\n");
        exit(1);
    }
}
                                
static void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
        unsigned char *pout;
        unsigned int   i;
        unsigned char  bitbuf[fsk->Nbits];
        
	if (ctx) {
		if (do_exit)
			return;

		if ((bytes_to_read > 0) && (bytes_to_read < len)) {
			len = bytes_to_read;
			do_exit = 1;
			rtlsdr_cancel_async(dev);
		}

                /*
		if (fwrite(buf, 1, len, (FILE*)ctx) != len) {
			fprintf(stderr, "Short write, samples lost, exiting!\n");
			rtlsdr_cancel_async(dev);
		}
                */

                /* append to existing samples in rawbuf */
                memcpy(&rawbuf[nrawbuf],buf,len);
                nrawbuf += len;
                assert(nrawbuf < 2*out_block_size);

                /* process as many samples in rawbuf as possible */
                pout = rawbuf;
                while(nrawbuf >= 2*fsk_nin(fsk)) {
                    COMP modbuf[fsk->N+fsk->Ts*2];  /* max nin value with timing slips */
                    /* complex unsigned 8 bit to float conversion */
                    for(i=0;i<fsk_nin(fsk);i++){
                        modbuf[i].real = ((float)pout[2*i]-127.0)/128.0;
                        modbuf[i].imag = ((float)pout[2*i+1]-127.0)/128.0;
                    }
                    pout += 2*fsk_nin(fsk);
                    nrawbuf -= 2*fsk_nin(fsk);
                    fsk_demod(fsk, bitbuf, modbuf);
                    if (fwrite(bitbuf, 1, fsk->Nbits, (FILE*)ctx) != (unsigned int)fsk->Nbits) {
			fprintf(stderr, "Short write, bits lost, exiting!\n");
			rtlsdr_cancel_async(dev);
                    }
                    if (udp_debug) {
                        sample_counter += fsk_nin(fsk);
                        if (sample_counter > samp_rate) {
                            char buf[BUF_SZ];
                            size_t Ndft;
                            /* one second has passed, lets send some debug information */
                            sample_counter -= samp_rate;

                            /* Print a sample of the FFT from the freq estimator */
                            snprintf(buf, BUF_SZ, "{"); udp_sendbuf(buf);
                            snprintf(buf, BUF_SZ, "\"Sf\":["); udp_sendbuf(buf);
                            Ndft = fsk->Ndft;
                            for(i=0; i<Ndft; i++) {
                                snprintf(buf, BUF_SZ, "%f ",(fsk->Sf)[i]); udp_sendbuf(buf);
                                if(i<Ndft-1) { snprintf(buf, BUF_SZ, ","); udp_sendbuf(buf); }
                            }
                            snprintf(buf, BUF_SZ, "]"); udp_sendbuf(buf);
                            /* TODO: tone freq ests, SNRest, maybe buffer sample clock offsets and send as an array */
                            snprintf(buf, BUF_SZ, "}\n"); udp_sendbuf(buf);
                            fprintf(stderr, "Tick\n");
                        }
                    }
                }

                /* copy left over to start of buffer */
                memmove(rawbuf,pout,nrawbuf);
                
		if (bytes_to_read > 0)
			bytes_to_read -= len;
	}
}

int main(int argc, char **argv)
{
#ifndef _WIN32
	struct sigaction sigact;
#endif
	char *filename = NULL;
	int n_read;
	int r, opt;
	int gain = 0;
	int ppm_error = 0;
	int sync_mode = 0;
	FILE *file;
	uint8_t *buffer;
	int dev_index = 0;
	int dev_given = 0;
	uint32_t frequency = 100000000;

	while ((opt = getopt(argc, argv, "d:f:g:s:b:n:p:S:u:")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = verbose_device_search(optarg);
			dev_given = 1;
			break;
		case 'f':
			frequency = (uint32_t)atofs(optarg);
			break;
		case 'g':
			gain = (int)(atof(optarg) * 10); /* tenths of a dB */
			break;
		case 's':
			samp_rate = (uint32_t)atofs(optarg);
			break;
		case 'p':
			ppm_error = atoi(optarg);
			break;
		case 'b':
			out_block_size = (uint32_t)atof(optarg);
			break;
		case 'n':
			bytes_to_read = (uint32_t)atof(optarg) * 2;
			break;
		case 'S':
			sync_mode = 1;
			break;
		case 'u':
                        udp_debug = 1;
                        strcpy(hostname, optarg);
			break;
		default:
			usage();
			break;
		}
	}

	if (argc <= optind) {
		usage();
	} else {
		filename = argv[optind];
	}

	if(out_block_size < MINIMAL_BUF_LENGTH ||
	   out_block_size > MAXIMAL_BUF_LENGTH ){
		fprintf(stderr,
			"Output block size wrong value, falling back to default\n");
		fprintf(stderr,
			"Minimal length: %u\n", MINIMAL_BUF_LENGTH);
		fprintf(stderr,
			"Maximal length: %u\n", MAXIMAL_BUF_LENGTH);
		out_block_size = DEFAULT_BUF_LENGTH;
	}

	buffer = malloc(out_block_size * sizeof(uint8_t));
	rawbuf = malloc(2 * out_block_size * sizeof(uint8_t));
        nrawbuf = 0;

        /* create UDP socket for debug/status information */
        if (udp_debug) {
            struct hostent *server;
            
            /* socket: create the socket */
            sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            if (sockfd < 0) {
                fprintf(stderr,"ERROR opening socket");
                exit(1);
            }

            /* gethostbyname: get the server's DNS entry */
            server = gethostbyname(hostname);
            if (server == NULL) {
                fprintf(stderr,"ERROR, no such host as %s\n", hostname);
                exit(1);
            }

            /* build the server's Internet address */
            bzero((char *) &serveraddr, sizeof(serveraddr));
            serveraddr.sin_family = AF_INET;
            bcopy((char *)server->h_addr, 
                  (char *)&serveraddr.sin_addr.s_addr, server->h_length);
            serveraddr.sin_port = htons(portno);
        }

        /* continue RTL SDR setup ...... */
        
	if (!dev_given) {
		dev_index = verbose_device_search("0");
	}

	if (dev_index < 0) {
		exit(1);
	}

	r = rtlsdr_open(&dev, (uint32_t)dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
		exit(1);
	}
#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif
	/* Set the sample rate */
	verbose_set_sample_rate(dev, samp_rate);

	/* Set the frequency */
	verbose_set_frequency(dev, frequency);

	if (0 == gain) {
		 /* Enable automatic gain */
		verbose_auto_gain(dev);
	} else {
		/* Enable manual gain */
		gain = nearest_gain(dev, gain);
		verbose_gain_set(dev, gain);
	}

	verbose_ppm_set(dev, ppm_error);

	if(strcmp(filename, "-") == 0) { /* Write samples to stdout */
		file = stdout;
#ifdef _WIN32
		_setmode(_fileno(stdin), _O_BINARY);
#endif
	} else {
		file = fopen(filename, "wb");
		if (!file) {
			fprintf(stderr, "Failed to open %s\n", filename);
			goto out;
		}
	}
        
        {
            /* TODO: make some of these command line options */
            int Fs = DEFAULT_SAMPLE_RATE;
            int Rs = 10000;
            int M  = 2;
            int P  = Fs/Rs;
            fsk = fsk_create_hbr(Fs,Rs,M,P,FSK_DEFAULT_NSYM,FSK_NONE,100);
        }
        {
            /* TODO: fsk_lower might need some adjustment, so we avoid the RTLSDR DC line.  We really need a GUI
               plot of the spectrum and tone estimates to observe what's going on */
            int fsk_lower = 0;
            int fsk_upper = DEFAULT_SAMPLE_RATE/2;
            fprintf(stderr,"Setting estimator limits to %d to %d Hz.\n", fsk_lower, fsk_upper);
            fsk_set_freq_est_limits(fsk,fsk_lower,fsk_upper);
        }
        /* Reset endpoint before we start reading from it (mandatory) */
	verbose_reset_buffer(dev);

	if (sync_mode) {
		fprintf(stderr, "Reading samples in sync mode...\n");
		while (!do_exit) {
			r = rtlsdr_read_sync(dev, buffer, out_block_size, &n_read);
			if (r < 0) {
				fprintf(stderr, "WARNING: sync read failed.\n");
				break;
			}

			if ((bytes_to_read > 0) && (bytes_to_read < (uint32_t)n_read)) {
				n_read = bytes_to_read;
				do_exit = 1;
			}

			if (fwrite(buffer, 1, n_read, file) != (size_t)n_read) {
				fprintf(stderr, "Short write, samples lost, exiting!\n");
				break;
			}

			if ((uint32_t)n_read < out_block_size) {
				fprintf(stderr, "Short read, samples lost, exiting!\n");
				break;
			}

			if (bytes_to_read > 0)
				bytes_to_read -= n_read;
		}
	} else {
		fprintf(stderr, "Reading samples in async mode...\n");
		r = rtlsdr_read_async(dev, rtlsdr_callback, (void *)file,
				      0, out_block_size);
	}

	if (do_exit)
		fprintf(stderr, "\nUser cancel, exiting...\n");
	else
		fprintf(stderr, "\nLibrary error %d, exiting...\n", r);

	if (file != stdout)
		fclose(file);

	rtlsdr_close(dev);
        fsk_destroy(fsk);
	free (buffer);
	free (rawbuf);
out:
	return r >= 0 ? r : -r;
}
