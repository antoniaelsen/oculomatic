#ifndef SRC_COMMS_COMEDI_
#define SRC_COMMS_COMEDI_

#ifdef USE_COMEDI

#include <comedilib.h>


#define SAMPLE_CT 5 // about as short as you can get
#define BUF_LEN 0x8000


struct parsed_options_hold {
  char filename[256];
  float value;
  int subdevice;
  int channel;
  int aref;
  int range;
  int physical;
  int verbose;
  int n_chan;
  int n_scan;
  float freq;
};

class ComediClient {
 public:
  ComediClient();
  void dump_cmd(FILE *out,comedi_cmd *cmd);
  int write(comedi_t* device, int subdevice, int channelx, int channely, lsampl_t* dataxl, lsampl_t* datayl, int range, int aref);

 private:
  int external_trigger_number = 0;
  comedi_t *devx;
  comedi_t *devy;
  int subdevicex = -1;
  int subdevicey = -1;

  // Setup comedi
  comedi_cmd cmdx;
  comedi_cmd cmdy;
  int err;
  int n,m, i;
  int total=0, n_chan = 0, freq = 80000;
  int verbose = 0;
  unsigned int chanlistx[2];
  unsigned int chanlisty[1];
  unsigned int maxdata_x;
  unsigned int maxdata_y;
  comedi_range *rng_x;
  comedi_range *rng_y;
  int ret;
  //struct parsed_options options;
  int fn;
  int aref = AREF_GROUND;
  int range = 0;
  int channelx = 0;
  int channely = 1;
  int buffer_length;

  n_chan = 2;

  char* cmd_src(int src, char *buf);
};

#endif  // USE_COMEDI

#endif  // SRC_COMMS_COMEDI_
