//
// AppConfig.h
// parsing program options and arguments.
//
// @Yu

struct AppConfig
{
	char* inputDirectory;
	bool verbose;
	AppConfig() {
		inputDirectory = NULL;
		verbose = false;
	}
};

void print_usage(const char* binaryName);
void print_usage_abort(const char* binaryName);
AppConfig parseArgs(int argc, char **argv);



