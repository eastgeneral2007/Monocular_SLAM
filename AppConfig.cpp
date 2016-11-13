//
// AppConfig.cpp
// parsing program options and arguments.
//
// @Yu

#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <unistd.h>
#include "AppConfig.h"

void print_usage(const char* binaryName)
{
	printf("\n");
	printf("Usage: %s [options] <input_frame_directory>\n", binaryName);
	printf("Program Options:\n");
	printf("  -v                print verbose info\n");
	printf("\n");
}

void print_usage_abort(const char* binaryName) {
	print_usage(binaryName);
	exit(-1);
}

AppConfig parseArgs(int argc, char **argv)
{
	AppConfig config;
	opterr = 0;
	int c;
	while ((c = getopt(argc, argv, "v")) != -1)
	{
		switch(c) 
		{
			case 'v':
			config.verbose = true;
			break;
			case '?':
			if (optopt == 'i') {
				fprintf(stderr, "input dataset is not specified.\n");
			}
			else if(isprint(optopt)) {
				fprintf(stderr, "unknown option: %c\n", optopt);
			}
			else {
				fprintf(stderr, "unknown option: \\x%x\n", optopt);
			}
			print_usage_abort(argv[0]);
		}
	}

	if (optind < argc) {
		config.inputDirectory = argv[optind];
		optind ++;
	}

	if (config.inputDirectory == NULL) {
		fprintf(stderr, "input dataset is not specified\n");
		print_usage_abort(argv[0]);
	}

	if (optind < argc) {
		for (int i = optind; i < argc; i++) {
			printf("Non-option argument: %s\n", argv[i]);
			print_usage_abort(argv[0]);
		}
	}
	return config;
}