  // // Setup comedi
  // comedi_cmd cmdx;
  // comedi_cmd cmdy;
  // int err;
  // int n,m, i;
  // int total=0, n_chan = 0, freq = 80000;
  // int subdevicex = -1;
  // int subdevicey = -1;
  // int verbose = 0;
  // unsigned int chanlistx[2];
  // unsigned int chanlisty[1];
  // unsigned int maxdata_x;
  // unsigned int maxdata_y;
  // comedi_range *rng_x;
  // comedi_range *rng_y;
  // int ret;
  // //struct parsed_options options;
  // int fn;
  // int aref = AREF_GROUND;
  // int range = 0;
  // int channelx = 0;
  // int channely = 1;
  // int buffer_length;
  // subdevicex = -1;
  // subdevicey = -1;

  // n_chan = 2;

  // Comedi device setup
  // devx = comedi_open(comdevice);
  // devy = comedi_open(comdevice2);
  // if(devx == NULL){
  //   fprintf(stderr, "error opening %s\n", comdevice);
  //   return -1;
  // }

  // if(devy == NULL){
  //   fprintf(stderr,"error opening %s\n", comdevice2);
  //   return -1;
  // // }

  // if(subdevicex <0)
  // subdevicex = comedi_find_subdevice_by_type(devx, COMEDI_SUBD_AO, 0);
  // assert(subdevicex >= 0);

  // if(subdevicey <0)
  // subdevicey = comedi_find_subdevice_by_type(devy, COMEDI_SUBD_AO, 0);
  // assert(subdevicey >= 0);

  // maxdata_x = comedi_get_maxdata(devx, subdevicex, channelx);
  // rng_x = comedi_get_range(devx, subdevicex, channelx, 0);
  // max_rngx = maxdata_x;

  // maxdata_y = comedi_get_maxdata(devy, subdevicey, channely);
  // rng_y = comedi_get_range(devy, subdevicey, channely, 0);
  // max_rngy = maxdata_y;


      // //output to analog voltage
      // ret = comedi_internal_trigger_cust(devx,subdevicex,channelx, channely,dataxl,datayl,range,aref);