// #include <comedilib.h>


// Initialize
// comedi_t *devx;
// comedi_t *devy;
#define SAMPLE_CT 5 // about as short as you can get
#define BUF_LEN 0x8000
// #define COMEDI_DEVICE_AO "/dev/comedi0"

// const char *comdevice = COMEDI_DEVICE_AO;
// const char *comdevice2 = COMEDI_DEVICE_AO;

// int external_trigger_number = 0;

// lsampl_t dataxl[SAMPLE_CT];
// lsampl_t datayl[SAMPLE_CT];


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

// char *cmd_src(int src,char *buf)
// {
//   buf[0]=0;

//   if(src&TRIG_NONE)strcat(buf,"none|");
//   if(src&TRIG_NOW)strcat(buf,"now|");
//   if(src&TRIG_FOLLOW)strcat(buf, "follow|");
//   if(src&TRIG_TIME)strcat(buf, "time|");
//   if(src&TRIG_TIMER)strcat(buf, "timer|");
//   if(src&TRIG_COUNT)strcat(buf, "count|");
//   if(src&TRIG_EXT)strcat(buf, "ext|");
//   if(src&TRIG_INT)strcat(buf, "int|");
//   if(src&TRIG_OTHER)strcat(buf, "other|");

//   if(strlen(buf)==0){
//     sprintf(buf,"unknown(0x%08x)",src);
//   }else{
//     buf[strlen(buf)-1]=0;
//   }

//   return buf;
// }


// Comedi script for 2 channel analog output
// int comedi_internal_trigger_cust(comedi_t* device, int subdevice, int channelx, int channely, lsampl_t* dataxl, lsampl_t* datayl, int range, int aref)
// {

//   comedi_insn insn[2];
//   comedi_insnlist il;

//   il.n_insns=2;
//   il.insns=insn;

//   memset(&insn[0], 0, sizeof(comedi_insn));
//   insn[0].insn = INSN_WRITE; //INSN_INTTRIG
//   insn[0].subdev = subdevice;
//   insn[0].data = dataxl;
//   insn[0].n = SAMPLE_CT;
//   insn[0].chanspec = CR_PACK(channelx,range,aref);

//   memset(&insn[1], 0, sizeof(comedi_insn));
//   insn[1].insn = INSN_WRITE; //INSN_INTTRIG
//   insn[1].subdev = subdevice;
//   insn[1].data = datayl;
//   insn[1].n = SAMPLE_CT;
//   insn[1].chanspec = CR_PACK(channely,range,aref);

//   return comedi_do_insnlist(device, &il);
// }


// void dump_cmd(FILE *out,comedi_cmd *cmd)
// {
//   char buf[10];

//   fprintf(out,"subdevice:      %d\n",
//   cmd->subdev);

//   fprintf(out,"start:      %-8s %d\n",
//   cmd_src(cmd->start_src,buf),
//   cmd->start_arg);

//   fprintf(out,"scan_begin: %-8s %d\n",
//   cmd_src(cmd->scan_begin_src,buf),
//   cmd->scan_begin_arg);

//   fprintf(out,"convert:    %-8s %d\n",
//   cmd_src(cmd->convert_src,buf),
//   cmd->convert_arg);

//   fprintf(out,"scan_end:   %-8s %d\n",
//   cmd_src(cmd->scan_end_src,buf),
//   cmd->scan_end_arg);

//   fprintf(out,"stop:       %-8s %d\n",
//   cmd_src(cmd->stop_src,buf),
//   cmd->stop_arg);
// }

