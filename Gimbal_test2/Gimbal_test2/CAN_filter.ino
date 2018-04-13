void CAN_filter(){
  //filter setup
  CAN_filter_t mask;
  mask.id = 0x7FF;
  mask.ext = 0;
  mask.rtr = 0;

  CAN_filter_t Filter1;
  Filter1.id = 0x201;
  Filter1.ext = 0;
  Filter1.rtr = 0;
  CAN_filter_t Filter2;
  Filter2.id = 0x202;
  Filter2.ext = 0;
  Filter2.rtr = 0;
  CAN_filter_t Filter3;
  Filter3.id = 0x203;
  Filter3.ext = 0;
  Filter3.rtr = 0;
  CAN_filter_t Filter4;
  Filter4.id = 0x204;
  Filter4.ext = 0;
  Filter4.rtr = 0;
  CAN_filter_t Filter5;
  Filter5.id = 0x205;
  Filter5.ext = 0;
  Filter5.rtr = 0;
  CAN_filter_t Filter6;
  Filter6.id = 0x206;
  Filter6.ext = 0;
  Filter6.rtr = 0;
  CAN_filter_t Filter7;
  Filter7.id = 0x206;
  Filter7.ext = 0;
  Filter7.rtr = 0;
  CAN_filter_t Filter8;
  Filter8.id = 0x206;
  Filter8.ext = 0;
  Filter8.rtr = 0;
  
  CANbus.begin(mask);
  CANbus.setFilter(Filter1, 0);
  CANbus.setFilter(Filter2, 1);
  CANbus.setFilter(Filter3, 2);
  CANbus.setFilter(Filter4, 3);
  CANbus.setFilter(Filter5, 4);
  CANbus.setFilter(Filter6, 5);
  CANbus.setFilter(Filter7, 6);
  CANbus.setFilter(Filter8, 7);
}
