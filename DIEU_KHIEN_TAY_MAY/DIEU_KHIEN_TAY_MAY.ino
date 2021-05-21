#include "ModbusMaster.h"
#include "Keypad5x7.h"
#include "EEPROM_25LC256.h"
#include "ST7920_MCP23017_GLCD.h"
#include "TAYMAY.h"

#define ARDUINO_SLAVE_ADDRESS       2    // Dia chi board arduino slaver can dieu khien
#define MCP23017_I2C_ADDRESS        0x40 // MCP23017_I2C_ADDRESS 0x20 + 0 = 0x40
#define SELECT_EEPROM_25LC256_01    53
#define SELECT_EEPROM_25LC256_02    54

#define LCD_PWM 23
#define MAX_CHAR_01 8
//===========================================================
uint16_t Program_start_addr = 0xFFFF; // Địa chỉ bắt đầu chạy khi nhấn nút Auto
uint16_t prog_name;
bool choose_program = false;
bool choose_teach_mode = false;
uint16_t COMMAND = 0xFFFF;
uint16_t COMMAND_MANUAL = 0xFFFF;
uint16_t data;

ModbusMaster            NODE;
Keypad5x7               KEYPAD;
EEPROM_25LC256          EEPROM256;
ST7920_MCP23017_GLCD    GLCD;

void Show_Infor(void){
  GLCD.clcd_init_text_mode();
  GLCD.text_display_clear();
  GLCD.printTxt(0,0,"KHOI NGUYEN .JSC"); delay(1);
  GLCD.printTxt(1,0,"INJECTION ROBOT"); delay(1);
  GLCD.printTxt(2,0,"Design:KHANH.Q.N"); delay(1);
  GLCD.printTxt(3,0,"Phone 0348777219"); delay(1);
}
//================================================================
//================================================================
void Remove_Pointer_Program(int x, int y){
  //=======================================
  if (x == 0 && y == 0) { GLCD.printTxt(1,0," ");}
  if (x == 1 && y == 0) { GLCD.printTxt(1,3," ");}
  if (x == 2 && y == 0) { GLCD.printTxt(1,6," ");}
//=======================================
  if (x == 0 && y == 1) { GLCD.printTxt(2,0," ");}
  if (x == 1 && y == 1) { GLCD.printTxt(2,3," ");}
  if (x == 2 && y == 1) { GLCD.printTxt(2,6," ");} 
//=======================================
  if (x == 0 && y == 2) { GLCD.printTxt(3,0," ");}
  if (x == 1 && y == 2) { GLCD.printTxt(3,3," ");}
  if (x == 2 && y == 2) { GLCD.printTxt(3,6," ");}
}
//============================================================================================
// Hiện thị con trỏ trước tên chương trình
uint8_t Show_Pointer_Program(int x, int y){
  static int8_t x_prev = 0; static int8_t y_prev = 0;
  uint8_t pos = 0xFF;
  if (x_prev != x || y_prev != y){
    Remove_Pointer_Program(x_prev, y_prev); x_prev = x; y_prev = y;
  }
//=======================================
  if (x == 0 && y == 0) { GLCD.printTxt(1,0,">"); pos = 1;}
  if (x == 1 && y == 0) { GLCD.printTxt(1,3,">"); pos = 2;}
  if (x == 2 && y == 0) { GLCD.printTxt(1,6,">"); pos = 3;}
//=======================================
  if (x == 0 && y == 1) { GLCD.printTxt(2,0,">"); pos = 4;}
  if (x == 1 && y == 1) { GLCD.printTxt(2,3,">"); pos = 5;}
  if (x == 2 && y == 1) { GLCD.printTxt(2,6,">"); pos = 6;} 
//=======================================
  if (x == 0 && y == 2) { GLCD.printTxt(3,0,">"); pos = 7;}
  if (x == 1 && y == 2) { GLCD.printTxt(3,3,">"); pos = 8;}
  if (x == 2 && y == 2) { GLCD.printTxt(3,6,">"); pos = 9;}
  return pos;
}
//============================================================================================
//============================================================================================
void Remove_Pointer_Function(int x, int y){
//=======================================
  if (x == 0 && y == 0) { GLCD.printTxt(1,0," ");}
  if (x == 1 && y == 0) { GLCD.printTxt(1,4," ");}
//=======================================
  if (x == 0 && y == 1) { GLCD.printTxt(2,0," ");}
  if (x == 1 && y == 1) { GLCD.printTxt(2,4," ");}
//=======================================
  if (x == 0 && y == 2) { GLCD.printTxt(3,0," ");}
  if (x == 1 && y == 2) { GLCD.printTxt(3,4," ");}
}
//============================================================================================
// Hiện thị con trỏ trước tên chương trình
uint8_t Show_Pointer_Function(int x, int y){
  static int8_t x_prev = 0; static int8_t y_prev = 0;
  uint8_t pos = 0xFF;
  if (x_prev != x || y_prev != y){
    Remove_Pointer_Function(x_prev, y_prev);  x_prev = x; y_prev = y;
  }
//=======================================
  if (x == 0 && y == 0) { GLCD.printTxt(1,0,">"); pos = 1;}
  if (x == 1 && y == 0) { GLCD.printTxt(1,4,">"); pos = 2;}
//=======================================
  if (x == 0 && y == 1) { GLCD.printTxt(2,0,">"); pos = 3;}
  if (x == 1 && y == 1) { GLCD.printTxt(2,4,">"); pos = 4;}
//=======================================
  if (x == 0 && y == 2) { GLCD.printTxt(3,0,">"); pos = 5;}
  if (x == 1 && y == 2) { GLCD.printTxt(3,4,">"); pos = 6;}
  return pos;
}
//============================================================================================

// Truyen Program_start_addr tới board slaver
// Set bit thứ PROGRAM_START_MODBUS_ADDR của COIL[] board slaver để láy giá trị state_coil cần command
// value: Giá trị địa chỉ bắt đầu đọc của eeprom slave board.

void Set_program_start_addr_to_slave_board(uint16_t value){
    NODE.writeSingleCoil(PROGRAM_START_MODBUS_ADDR, CHOOSE);   // Set COIL[PROGRAM_START_MODBUS_ADDR] 
    NODE.writeSingleRegister(SINGLE_REGISTER_ADDR,value);
    NODE.writeSingleCoil(PROGRAM_START_MODBUS_ADDR, NOCHOOSE); // Reset COIL[PROGRAM_START_MODBUS_ADDR]
}
//============================================================================================
// Truyen command chọn eeprom tới board slaver
// Set bit thứ CHOOSE_EEPROM_MODBUS_ADDR của COIL[] board slaver để láy giá trị state_coil cần command
// choose_eeprom: chọn eeprom 1 hoặc eeprom 2.
void Set_eeprom_type_to_slave_board(uint16_t choose_eeprom){
    NODE.writeSingleCoil(CHOOSE_EEPROM_MODBUS_ADDR, CHOOSE);   // Set COIL[CHOOSE_EEPROM_MODBUS_ADDR] 
    NODE.writeSingleRegister(SINGLE_REGISTER_ADDR,choose_eeprom);
    NODE.writeSingleCoil(CHOOSE_EEPROM_MODBUS_ADDR, NOCHOOSE); // Reset COIL[CHOOSE_EEPROM_MODBUS_ADDR]
}
//============================================================================================
// Ghi giá trị 1 word value vào eeprom board slave bắt đầu ở địa chỉ addr
// Kiểm tra giá trị ghi vào eeprom board slave đúng hay chưa
uint8_t Write_single_data_to_Eeprom_slave_board(uint16_t value, uint16_t addr){
    uint16_t value_read; uint8_t status = 1; 
    //Serial.println("write_eeprom_status: value_write: "); Serial.println(value,HEX);
    NODE.writeSingleCoil(SINGLE_REG_MODBUS_ADDR, CHOOSE);   // Set COIL[SINGLE_REG_MODBUS_ADDR] 
    NODE.writeSingleRegister(addr,value);
    NODE.writeSingleCoil(SINGLE_REG_MODBUS_ADDR, NOCHOOSE); // Reset COIL[SINGLE_REG_MODBUS_ADDR]
    delay(100); // Delay quan trọng để quá trình ghi data vào eeprom hoàn tất trước khi đọc data ngược lại
    /* Kiểm tra giá trị ghi vào eeprom board slave đã đúng chưa bẳng cách read lại giá trị từ eeprom */
    value_read = Read_single_data_from_Eeprom_slave_board(addr);
    //Serial.print("write_eeprom_status: value_read: "); Serial.println(value_read); Serial.println(addr,HEX);
    if (value_read == value) {status = 0;}
    Serial.print("write_2 byte_to_eeprom_status: "); Serial.println(status);
    return status;
}
//============================================================================================
// Đọc giá trị 1 word data từ eeprom board slave bắt đầu ở địa chỉ addr
uint16_t Read_single_data_from_Eeprom_slave_board(uint16_t addr){
    uint8_t result = 1;
    NODE.writeSingleCoil(READ_EEPROM_MODBUS_ADDR, CHOOSE);
    while (result){
      result = NODE.readHoldingRegisters(addr, 1);  // Đọc 1 word data bắt đầu ở địa chỉ addr
      if (result != 0){ result = 1;} 
      else { break;}
    }
    uint16_t data_receive = NODE.getResponseBuffer(0);
    NODE.writeSingleCoil(READ_EEPROM_MODBUS_ADDR, NOCHOOSE);
    return data_receive;
}
//============================================================================================
// Ghi data vào eeprom của board slave
// Set bit thứ epprom_type của COIL[] board slaver để chọn đối tượng cần ghi giá trị
// index: số word cần write
// addr: địa chỉ bắt đầu write
uint8_t Write_packet_data_to_Eeprom_slave_board(uint16_t *data, uint8_t index, uint16_t addr){
    uint8_t result = 1; uint8_t status = 0;
    uint16_t *value_read;
    NODE.writeSingleCoil(WRITE_EEPROM_MODBUS_ADDR, CHOOSE); // Set COIL[epprom_type]
    for (int i = 0; i < index; i++){
      NODE.setTransmitBuffer(i, data[i]);
    }
    while (result){
      result = NODE.writeMultipleRegisters(addr, index);
      if (result != 0){ result = 1;} 
      else { break;}
    }
    NODE.writeSingleCoil(WRITE_EEPROM_MODBUS_ADDR, NOCHOOSE);
    delay(200); // delay quan trọng để hoàn tất quá trình ghi data trước khi đọc ngược lại
    value_read = Read_packet_data_from_Eeprom_slave_board(index, addr);
    for (int i = 0; i < index; i++){
      Serial.print(value_read[i],HEX); Serial.print(" "); Serial.println(data[i],HEX);
      if (value_read[i] != data[i]) { status = 1; break; }
    }
    Serial.print("write packet data_to_eeprom_status: "); Serial.println(status);
  return status;
}
//============================================================================================
// Đọc data eeprom của board slave
// Set bit thứ epprom_type của COIL[] board slaver để chọn đối tượng cần đọc giá trị
// index: số word cần đọc
// addr: địa chỉ bắt đầu đọc
uint16_t *Read_packet_data_from_Eeprom_slave_board(uint8_t index, uint16_t addr){
    static uint16_t data_receive[MAX_FARME_DATASIZE]; // Khai báo static uint16_t *data_receive bị lỗi
    uint8_t result = 1;
    NODE.writeSingleCoil(READ_EEPROM_MODBUS_ADDR, CHOOSE);  // Set COIL[epprom_type]
    while (result){
      result = NODE.readHoldingRegisters(addr, index);
      if (result != 0){ result = 1;} 
      else { break;}
    }
    for (int i = 0; i< index; i++){
      data_receive[i] = NODE.getResponseBuffer(i);
    }
    NODE.writeSingleCoil(READ_EEPROM_MODBUS_ADDR, NOCHOOSE);
    return data_receive;
}
//============================================================================================
// Ghi data vào register output của board slave
// Set bit thứ register_name của COIL[] board slaver để chọn đối tượng cần ghi giá trị
// index: số word cần write
// addr: địa chỉ bắt đầu write: 0
void Write_packet_data_to_output_slave_board(uint16_t *data, uint8_t index){
    uint8_t result = 1;
    NODE.writeSingleCoil(MULTI_REG_MODBUS_ADDR, CHOOSE); // Set COIL[epprom_type]
    for (int i = 0; i < index; i++){
      NODE.setTransmitBuffer(i, data[i]);
    }
    while (result){
      result = NODE.writeMultipleRegisters(SINGLE_REGISTER_ADDR, index);
      if (result != 0){ result = 1;} 
      else { break;}
    }
    NODE.writeSingleCoil(MULTI_REG_MODBUS_ADDR, NOCHOOSE);
}
//============================================================================================
// Đọc data từ feedback register của board slave để monitor
// index: số word cần read
// addr: địa chỉ bắt đầu read: SINGLE_REGISTER_ADDR
uint16_t *Read_packet_data_from_feedback_register_slave_board(uint8_t index){
    static uint16_t data_receive[MAX_FARME_DATASIZE]; // Khai báo static uint16_t *data_receive bị lỗi
    uint8_t result = 1;
    NODE.writeSingleCoil(READ_REGISTER_MODBUS_ADDR, CHOOSE);  // Set COIL[epprom_type]
    while (result){
      result = NODE.readHoldingRegisters(SINGLE_REGISTER_ADDR, index);

      if (result != 0){ result = 1;} 
      else { break;}
    }
    for (int i = 0; i< index; i++){
      data_receive[i] = NODE.getResponseBuffer(i);
    }
    NODE.writeSingleCoil(READ_REGISTER_MODBUS_ADDR, NOCHOOSE);
    return data_receive;
}
//============================================================================================
//============================================================================================
void setup() {  
  Serial.begin(9600);
  MODBUS_SERIAL.begin(MODBUS_BAUDRATE);

  NODE.begin(ARDUINO_SLAVE_ADDRESS, MODBUS_SERIAL);
  KEYPAD.begin();
  EEPROM256.begin(SELECT_EEPROM_25LC256_01);
  GLCD.init_display(MCP23017_I2C_ADDRESS);

  pinMode(LCD_PWM, OUTPUT); digitalWrite(LCD_PWM, LOW);
  Show_Infor();
  //Init_timer_counter_01();
}
//============================================================================================
//============================================================================================
void loop() {
  COMMAND = KEYPAD.Switch_press_scan();
  Serial.println(COMMAND,HEX);
//============================================================================================
  if (COMMAND == OPERATION_01 || COMMAND == OPERATION_02) {
    
    int8_t x = 0; int8_t y = 0; uint8_t program; uint8_t row,col; 
    uint16_t program_name[Program_number_max]; uint16_t program_step[Program_number_max];
    bool OPERATION_01_MODE = false; bool OPERATION_02_MODE = false; 
    choose_program = false; choose_teach_mode = false;

    GLCD.text_display_clear(); delay(1);

    switch (COMMAND) {
      case OPERATION_01: 
          OPERATION_01_MODE = true; 
          GLCD.printTxt(0,0,"OP.0"); delay(1); Set_eeprom_type_to_slave_board(CHOOSE_EEPROM_01);
          break;

      case OPERATION_02: 
          OPERATION_02_MODE = true;  
          GLCD.printTxt(0,0,"OP.1"); delay(1); Set_eeprom_type_to_slave_board(CHOOSE_EEPROM_02);
          break;
    }
    COMMAND = 0xFFFF;
    // Hiện thị tên các chương trình trong bộ nhớ EEPROM256
    // Nếu chương trình nào đã ready to autorun: stick dấu * trên tên chương trình
    row = 1; col = 0;
    for (int i = 0; i < Program_number_max; i++) {
        program_name[i] = Read_single_data_from_Eeprom_slave_board(Program_Addr[i]);     // Đọc số thứ tự chương trình
        program_step[i] = Read_single_data_from_Eeprom_slave_board(Program_Addr[i] + 2); // Đọc kiểm tra PROGRAM_NOT_READY/PROGRAM_READY
        GLCD.print_program_name(row,col,program_name[i],program_step[i]);
        col += 3;
        if (col == 9) { row ++; col = 0;}
    }
    while(OPERATION_02_MODE || OPERATION_01_MODE){
      for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++) { if (x == i && y == j) { program = Show_Pointer_Program(x,y); }}
      }
      COMMAND = KEYPAD.Switch_press_scan(); 
      
       switch (COMMAND) {
        case BUTTON_RIGHT:
          x++; if (x == 3)  {x = 2;} break;
        case BUTTON_LEFT:        
          x--; if (x == -1) {x = 0;} break;
        case BUTTON_UP:
          y--; if (y == -1) {y = 0;} break;
        case BUTTON_DOWN:
          y++; if (y == 3)  {y = 2;} break;
        case BUTTON_ENTER:{
            
           // Chọn chương trình để chạy
           // Lưu chương trình, truyền chương trình về board I/O qua bus RS485
          for (int i = 1; i < 10; i++) {
            if (program == i) { 
              Program_start_addr = Program_Addr[i - 1];
              Set_program_start_addr_to_slave_board(Program_start_addr);
              break;
            }
          }
            GLCD.text_display_clear(); delay(1);
            if (OPERATION_01_MODE == true) {GLCD.printTxt(0,0,"OP_0"); }
            if (OPERATION_02_MODE == true) {GLCD.printTxt(0,0,"OP_1"); }
            prog_name = program_name[program - 1];
            GLCD.printTxt(1,0,"CHUONG TRINH"); GLCD.print_number(1,6,prog_name);

            if (program_step[program - 1] != PROGRAM_READY) { // Chưa có chương trình
                choose_teach_mode = true;           // cho phép được vào teach mode
                GLCD.printTxt(3,3,"TEACH"); 
            } else {
              choose_program = true;    // cho phép chạy auto/modify/
              GLCD.printTxt(3,0,"AUTO/REMOTR/DEL");
              }                         // Serial.println(program_step[program - 1],HEX);                         
            OPERATION_01_MODE = OPERATION_02_MODE = false;
            break;
         }
        case OPERATION_01:
        case OPERATION_02:
           
           // Nếu nhấn phím operation_01 lần 2 sẽ thoát khỏi chế độ operation_01
           GLCD.text_display_clear();
           Show_Infor();
           choose_program = false; choose_teach_mode = false;
           OPERATION_01_MODE = OPERATION_02_MODE = false;
           break;
       } // END switch
      delay(1);
    }
  } // END OPERATION MODE
//============================================================================================ 
// Vào chế độ dạy, tạo chương trình mới
// Mỗi lệnh write giá trị vào epprom -> cần read lại giá trị đó để kiểm tra eeprom đã lưu đúng giá trị chưa.
//============================================================================================ 
if (COMMAND == TEACH_MODE) {
  
  while(choose_teach_mode == true) {            
    uint16_t index_address; uint8_t write_eeprom_status;
    bool state_mode = true; bool waiting_position = false;
    bool main_position = false; bool pick_position = false; 
    bool delay_step = false; bool state_vaccum = false; bool state_pick = false;

    GLCD.text_display_clear(); delay(1);
    GLCD.printTxt(0,0,"OP0 "); GLCD.printTxt(0,2,"PROG"); GLCD.print_number(0,4,prog_name);
    GLCD.printTxt(1,0, "PgDn:GotoNextPg"); 
    GLCD.printTxt(2,0, "Stop:ExitNoSave");
    GLCD.printTxt(3,0, "Teach:Exit&Save");

    while (COMMAND != BUTTON_PAGE_DOWN){COMMAND = KEYPAD.Switch_press_scan();}
   
    // Địa chỉ Program_start_addr lưu tên chương trình: 2 bytes 
    // Địa chỉ index_address = Program_start_addr + 2 lưu trạng thái PROGRAM_NOT_READY/PROGRAM_READY 
    index_address = Program_start_addr + 2;
    write_eeprom_status = Write_single_data_to_Eeprom_slave_board(PROGRAM_NOT_READY, index_address);

    if (write_eeprom_status != WRITE_DONE) {
      choose_teach_mode = false; 
      Display_eeprom_error(prog_name, index_address);
      break;
    }
/* Chọn mode arm cho tay máy: tùy vào từng mode arm mà cho phép set bit của từng mode arm tương ứng
ví dụ: + chọn mode main_arm là mode tay hút, lúc này chỉ có thế set các bit output coil thuộc mode tay hút.
các output coil thuộc mode tay kẹp sẽ không thể set được và ngược lại.
       + nếu chọn mode kết hợp, sẽ set được cả hai. ****** trường hợp này chưa lập trình ******
*/   
    index_address += 2;
    GLCD.text_display_clear(); delay(1);
    GLCD.printTxt(0,0,"N00:MODE"); GLCD.printTxt(0,4," PROG"); GLCD.print_number(0,6,prog_name);
    GLCD.printTxt(1,0,">Mode tay hut"); GLCD.printTxt(2,0," Mode tay gap"); GLCD.printTxt(3,0," Mode ket hop"); 
    uint16_t arm_mode[MODE_ARM_SIZE/2];
    arm_mode[FC_CODE] = MODE_ARM; arm_mode[FC_VALUE_W1] = MAIN_ARM_MODE; // Default Mode tay hut
    while (state_mode){ // Chọn mode
      COMMAND = KEYPAD.Switch_press_scan();
      
      switch (COMMAND) {

        case MAIN_ARM_MODE: arm_mode[FC_VALUE_W1] = MAIN_ARM_MODE; 
                            GLCD.printTxt(2,0," "); GLCD.printTxt(3,0," "); GLCD.printTxt(1,0,">Mode tay hut"); break;
        case PICK_ARM_MODE: arm_mode[FC_VALUE_W1] = PICK_ARM_MODE; 
                            GLCD.printTxt(1,0," "); GLCD.printTxt(3,0," "); GLCD.printTxt(2,0,">Mode tay gap"); break;
        case TWO_ARM_MODE:  arm_mode[FC_VALUE_W1] = TWO_ARM_MODE;  
                            GLCD.printTxt(1,0," "); GLCD.printTxt(2,0," "); GLCD.printTxt(3,0,">Mode ket hop"); break;
        case BUTTON_PAGE_DOWN: write_eeprom_status = Write_packet_data_to_Eeprom_slave_board(arm_mode, MODE_ARM_SIZE/2, index_address);
                                if (write_eeprom_status != WRITE_DONE) {
                                    choose_teach_mode = false; 
                                    Display_eeprom_error(prog_name, index_address);
                                    break;
                                }
                               index_address += MODE_ARM_SIZE;
                               state_mode = false; state_vaccum = true;
                               GLCD.text_display_clear(); delay(1); break;
        case STOP_01:
        case STOP_02:          state_mode = false; state_vaccum = false; waiting_position = false; main_position = false; 
                               state_pick = false; choose_teach_mode = false; 
                               GLCD.text_display_clear(); delay(1); Show_Infor(); break;
                               
        
      }
    }
    if (choose_teach_mode == false) { break; } // Thoát khỏi vòng lặp while choose_teach_mode

/* Chọn cảm biến chân không cho tay máy*/
    GLCD.printTxt(0,0,"N01:USE VACCUM");
    GLCD.printTxt(1,0,"Vacuum sensor"); GLCD.printTxt(2,0,">Use sensor"); GLCD.printTxt(3,0," No sensor"); 
    uint16_t choose_vaccum_sensor[VACCUM_SENSOR_SIZE/2];
    choose_vaccum_sensor[FC_CODE] = VACCUM_SENSOR; choose_vaccum_sensor[FC_VALUE_W1] = 0;
    while (state_vaccum) { // Chọn cảm biến chân không
      COMMAND = KEYPAD.Switch_press_scan();
      
      switch (COMMAND) {
        case ARM_VACUUM:   
                       
                      if (!choose_vaccum_sensor[FC_VALUE_W1]){GLCD.printTxt(2,0," "); GLCD.printTxt(3,0,">No sensor"); 
                          choose_vaccum_sensor[FC_VALUE_W1] = 1;}
                      else {GLCD.printTxt(3,0," "); GLCD.printTxt(2,0,">Use sensor"); 
                          choose_vaccum_sensor[FC_VALUE_W1] = 0;}

                      break;
        case BUTTON_PAGE_DOWN: 
                              
                              write_eeprom_status = Write_packet_data_to_Eeprom_slave_board(choose_vaccum_sensor, VACCUM_SENSOR_SIZE/2, index_address);
                               if (write_eeprom_status != WRITE_DONE) {
                                choose_teach_mode = false; 
                                Display_eeprom_error(prog_name, index_address);
                                break;
                               }
                               index_address += VACCUM_SENSOR_SIZE;
                               state_vaccum = false; state_pick = true; 
                               GLCD.text_display_clear(); delay(1); break;
        case STOP_01:
        case STOP_02:          state_vaccum = false; waiting_position = false; main_position = false; 
                               state_pick = false; choose_teach_mode = false; 
                               GLCD.text_display_clear(); delay(1); Show_Infor(); 
                               
                               break;
      }
    }
    if (choose_teach_mode == false) { break; } // Thoát khỏi vòng lặp while choose_teach_mode

/* Chọn cảm biến tay kẹp cho tay máy*/
    GLCD.printTxt(0,0,"N02:USE GRIP");
    GLCD.printTxt(1,0,"Grip sensor"); GLCD.printTxt(2,0,">Use sensor"); GLCD.printTxt(3,0," No sensor"); 
    uint16_t choose_pick_sensor[VACCUM_SENSOR_SIZE/2];
    choose_pick_sensor[FC_CODE] = PICK_SENSOR; choose_pick_sensor[FC_VALUE_W1] = 0;
    while (state_pick) { // Chọn cảm biến tay kẹp
      COMMAND = KEYPAD.Switch_press_scan();
      
      switch (COMMAND) {
        case ARM_VACUUM:    
                      if (!choose_pick_sensor[FC_VALUE_W1]){GLCD.printTxt(2,0," "); GLCD.printTxt(3,0,">No sensor"); 
                          choose_pick_sensor[FC_VALUE_W1] = 1;}
                      else {GLCD.printTxt(3,0," "); GLCD.printTxt(2,0,">Use sensor"); 
                          choose_pick_sensor[FC_VALUE_W1] = 0;}
                      break;
        case BUTTON_PAGE_DOWN: 
                                write_eeprom_status = Write_packet_data_to_Eeprom_slave_board(choose_pick_sensor, VACCUM_SENSOR_SIZE/2, index_address);
                               if (write_eeprom_status != WRITE_DONE) {
                                choose_teach_mode = false; 
                                Display_eeprom_error(prog_name, index_address);
                                break;
                               }
                               index_address += VACCUM_SENSOR_SIZE;
                               state_pick = false; waiting_position = true;
                               GLCD.text_display_clear(); delay(1); break;
        case STOP_01:
        case STOP_02:          state_pick = false; waiting_position = false; main_position = false; 
                               choose_teach_mode = false; 
                               GLCD.text_display_clear(); delay(1); Show_Infor(); break;
                               
      }
    }

    if (choose_teach_mode == false) { break; } // Thoát khỏi vòng lặp while choose_teach_mode

/* chọn vị trí chờ lấy sản phẩm */
    GLCD.printTxt(0,0,"N03:WAITING POS");  
    GLCD.printTxt(1,0," Tay xoay: "); GLCD.printTxt(2,0," Tay toi/lui");
    uint16_t output_coil[INPUT_OUTPUT_SIZE/2]; 
    output_coil[FC_CODE] = WAITING_POSSITON; output_coil[FC_VALUE_W1] = 0x0000; output_coil[FC_VALUE_W2] = 0x0000;
    // Set chế độ robot
    output_coil[FC_VALUE_W1] = bitSet(output_coil[FC_VALUE_W1], ROBOT_MODE);
    while (waiting_position) { // Khai báo vị trí chờ lấy sản phẩm

      COMMAND = KEYPAD.Switch_press_scan();
     
      switch (COMMAND) {

        case ARM_ROTATE_UP:   output_coil[FC_VALUE_W2] = bitSet(output_coil[FC_VALUE_W2],ARM_SET_ROTATE_UP);  
                              output_coil[FC_VALUE_W2] = bitClear(output_coil[FC_VALUE_W2],ARM_SET_ROTATE_DOWN);
                              Write_packet_data_to_output_slave_board(output_coil, INPUT_OUTPUT_SIZE/2);
                              GLCD.printTxt(1,0,">Tay xoay len  "); break;

        case ARM_ROTATE_DOWN: output_coil[FC_VALUE_W2] = bitSet(output_coil[FC_VALUE_W2],ARM_SET_ROTATE_DOWN);
                              output_coil[FC_VALUE_W2] = bitClear(output_coil[FC_VALUE_W2],ARM_SET_ROTATE_UP); 
                              Write_packet_data_to_output_slave_board(output_coil, INPUT_OUTPUT_SIZE/2);
                              GLCD.printTxt(1,0,">Tay xoay xuong");  break;

        case ARM_GO_LEFT:     output_coil[FC_VALUE_W2] = bitSet(output_coil[FC_VALUE_W2],ARM_SUB_LEFT_RIGHT);   
                              Write_packet_data_to_output_slave_board(output_coil, INPUT_OUTPUT_SIZE/2);
                              GLCD.printTxt(2,0,">Tay di toi ");  break;

        case ARM_GO_RIGHT:    output_coil[FC_VALUE_W2] = bitClear(output_coil[FC_VALUE_W2],ARM_SUB_LEFT_RIGHT);
                              Write_packet_data_to_output_slave_board(output_coil, INPUT_OUTPUT_SIZE/2);  
                              GLCD.printTxt(2,0,">Tay di lui "); break;
                                
        case BUTTON_PAGE_DOWN: 
                                  if (output_coil[FC_VALUE_W2] != 0){ // Nếu chưa chọn thì tiếp tục vòng lăp chọn waiting_position
                                  write_eeprom_status = Write_packet_data_to_Eeprom_slave_board(output_coil, INPUT_OUTPUT_SIZE/2, index_address);
                                      if (write_eeprom_status != WRITE_DONE) {
                                        choose_teach_mode = false; 
                                        Display_eeprom_error(prog_name, index_address);
                                        break;
                                      }
                                  index_address += INPUT_OUTPUT_SIZE;
                                  main_position = true; delay_step = true; waiting_position = false; 
                                  GLCD.text_display_clear(); delay(1);
                                } break;           
        case STOP_01:
        case STOP_02:           waiting_position = false; main_position = false; choose_teach_mode = false; 
                                GLCD.text_display_clear(); delay(1); Show_Infor(); break; 
      }
    } // end waiting position

    if (choose_teach_mode == false) { break; } // Thoát khỏi vòng lặp while choose_teach_mode

    uint16_t delay_value[STEP_REPEAT_DELAY_SIZE/2]; char delayArray[9]; int8_t count_value;
    delay_value[FC_CODE] = DELAY_STEP; 

    while (main_position){ // Các thao tác trong chương trình chính

      delay_value[FC_VALUE_W1] = 0; count_value = 0;
      GLCD.printTxt(0,0,"N04:DELAY POS");
      GLCD.printTxt(1,0,"Delay: "); GLCD.printTxt(1,3,"0s");
      GLCD.printTxt(3,0, "Press PgDn ...");
      for (int i = 0; i < 9; i++) {delayArray[i] = ' ';} delayArray[8] = '\0'; // Xóa giá trị delay value
    
      while (delay_step){
        COMMAND = KEYPAD.Switch_press_scan();
        
        switch (COMMAND) {
          case _1_:    delayArray[count_value++] = '1'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
          case _2_:    delayArray[count_value++] = '2'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
          case _3_:    delayArray[count_value++] = '3'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
          case _4_:    delayArray[count_value++] = '4'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
          case _5_:    delayArray[count_value++] = '5'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
          case _6_:    delayArray[count_value++] = '6'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
          case _7_:    delayArray[count_value++] = '7'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
          case _8_:    delayArray[count_value++] = '8'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
          case _9_:    delayArray[count_value++] = '9'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
          case _0_:    delayArray[count_value++] = '0'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
          case _point_:delayArray[count_value++] = '.'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;

          case BUTTON_DEL:  for (int i = 0; i < 9; i++) {delayArray[i] = ' ';} 
                            delayArray[8] = '\0'; GLCD.printTxt(1,3,delayArray);
                            GLCD.printTxt(1,3,"0s"); count_value = 0;
                            break; 
          case BUTTON_PAGE_DOWN:  delay_value[FC_VALUE_W1] = convert_str_to_delay_number(delayArray);
                                  write_eeprom_status = Write_packet_data_to_Eeprom_slave_board(delay_value, STEP_REPEAT_DELAY_SIZE/2, index_address);
                                  if (write_eeprom_status != WRITE_DONE) {
                                    choose_teach_mode = false; 
                                    Display_eeprom_error(prog_name, index_address);
                                    break;
                                  }
                                  index_address += STEP_REPEAT_DELAY_SIZE;
                                  pick_position = true; delay_step = false; 
                                  GLCD.text_display_clear(); delay(1); break;

          case TEACH_MODE:        pick_position = false; delay_step = false; main_position = false; choose_teach_mode = false;
                                  // Lưu mã kết thúc chương trình
                                  write_eeprom_status = Write_single_data_to_Eeprom_slave_board(REPEAT_STEP, index_address);
                                  if (write_eeprom_status != WRITE_DONE) {
                                      Display_eeprom_error(prog_name,index_address);
                                      break;
                                  }
                                  // Lưu mã sẵn sàng chạy auto/modify
                                  write_eeprom_status = Write_single_data_to_Eeprom_slave_board(PROGRAM_READY,  Program_start_addr + 2);
                                  if (write_eeprom_status != WRITE_DONE) {

                                      Display_eeprom_error(prog_name, Program_start_addr + 2);
                                      break;
                                  }
                                  GLCD.text_display_clear(); delay(1);
                                  GLCD.printTxt(1,0,"TEACH MODE DONE"); 
                                  GLCD.printTxt(3,0, "Press OP.0/OP.1");
                                  break;

          case STOP_01:           
          case STOP_02:           pick_position = false; delay_step = false; main_position = false; choose_teach_mode = false;
                                  GLCD.text_display_clear(); delay(1); Show_Infor(); break;

        }
      } //end delay_step

      if (choose_teach_mode == false) { break; } // Thoát khỏi vòng lặp while choose_teach_mode

      uint16_t output_coil_index[INPUT_OUTPUT_SIZE/2]; 
      uint8_t count_act = 1; // đếm số action 
      output_coil[FC_CODE] = OUTPUT_STEP;
      output_coil_index[FC_VALUE_W1] = output_coil[FC_VALUE_W1];
      output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2];

      while (pick_position) {
        GLCD.printTxt(0,0,"N05:DEFINE POS");
        GLCD.printTxt(1,0,"Action:"); GLCD.print_number(1,3,Number_STT[count_act]); 
        GLCD.printTxt(3,0, " PgDn/Enter/Ins");
        COMMAND = KEYPAD.Switch_press_scan();
        
        switch (COMMAND) {

          case ARM_ROTATE_UP:     output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2];
                                  output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_SET_ROTATE_UP);  
                                  output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_SET_ROTATE_DOWN);
                                  GLCD.printTxt(2,0,">Tay xoay len  "); break;

          case ARM_ROTATE_DOWN:   output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2];
                                  output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_SET_ROTATE_DOWN);
                                  output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_SET_ROTATE_UP);                                 
                                  GLCD.printTxt(2,0,">Tay xoay xuong"); break;

          case ARM_GO_LEFT:       output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2]; 
                                  output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_SUB_LEFT_RIGHT);                                  
                                  GLCD.printTxt(2,0,">Tay di toi    "); break;

          case ARM_GO_RIGHT:      output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2]; 
                                  output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_SUB_LEFT_RIGHT);                                 
                                  GLCD.printTxt(2,0,">Tay di lui    "); break;

          case ARM_UP:            output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2]; 
                                  output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_SUB_UP_DOWN);                                 
                                  GLCD.printTxt(2,0,">Tay di len    "); break;

          case ARM_DOWN:          output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2];
                                  output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_SUB_UP_DOWN);                                
                                  GLCD.printTxt(2,0,">Tay di xuong  "); break;

          case ARM_FORWARD:       output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2]; 
                                  output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_SET_FORWARD);
                                  output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_SET_REVERS);                                 
                                  GLCD.printTxt(2,0,">Toi vi tri tha"); break;

          case ARM_REVERS:        output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2];
                                  output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_SET_REVERS);
                                  output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_SET_FORWARD);                                 
                                  GLCD.printTxt(2,0,">Toi vi tri gap"); break;

          case ARM_PICK:          output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2]; 
                                  output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_PICK_UNPICK_01);
                                  output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_PICK_UNPICK_02);
                                  output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_SET_VACUUM_01);
                                  output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_SET_VACUUM_02);                                 
                                  GLCD.printTxt(2,0,">Lay san pham  "); break;

          case ARM_UNPICK:        output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2]; 
                                  output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_PICK_UNPICK_01);
                                  output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_PICK_UNPICK_02);
                                  output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_SET_VACUUM_01);
                                  output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_SET_VACUUM_02);                                 
                                  GLCD.printTxt(2,0,">Tha san pham  "); break;

          case ARM_VACUUM:        // Phát lệnh ép sản phẩm :đóng khuôn -> ép sản phẩm -> mở khuôn
                                  output_coil_index[FC_VALUE_W1] = output_coil[FC_VALUE_W1];
                                  output_coil_index[FC_VALUE_W1] = bitSet(output_coil_index[FC_VALUE_W1], MOLD_CLOSE);
                                  GLCD.printTxt(2,0,">EP SAN PHAM   "); break;

          case BUTTON_THANG:      // Khai báo OUTPUT_STEP là bước lấy sản phẩm
                                  output_coil_index[FC_VALUE_W1] = output_coil[FC_VALUE_W1];
                                  output_coil_index[FC_VALUE_W1] = bitSet(output_coil_index[FC_VALUE_W1], GET_PRODUCT);
                                  GLCD.printTxt(2,0,">LAY SAN PHAM  ");
                                  break;

          case BUTTON_INS:        // Chèn thêm output coil để thực hiện nhiều thao tác
                                  if ((output_coil[FC_VALUE_W2] != output_coil_index[FC_VALUE_W2]) || (output_coil[FC_VALUE_W1] != output_coil_index[FC_VALUE_W1])){
                                      count_act++;
                                      output_coil[FC_VALUE_W1] = output_coil_index[FC_VALUE_W1]; 
                                      output_coil[FC_VALUE_W2] = output_coil_index[FC_VALUE_W2];
                                      GLCD.text_display_clear(); delay(1);
                                  }break;

          case BUTTON_ENTER:      // Xuất output coil để mô phỏng thực tế những thao tác đã chọn
                                  Write_packet_data_to_output_slave_board(output_coil_index, INPUT_OUTPUT_SIZE/2); break;

          case BUTTON_PAGE_DOWN:  // Lưu giá trị output coil vào eeprom board slave
                                  if ((output_coil[FC_VALUE_W2] != output_coil_index[FC_VALUE_W2]) || (output_coil[FC_VALUE_W1] != output_coil_index[FC_VALUE_W1])){
                                      output_coil[FC_VALUE_W1] = output_coil_index[FC_VALUE_W1]; 
                                      output_coil[FC_VALUE_W2] = output_coil_index[FC_VALUE_W2];  
                                      write_eeprom_status = Write_packet_data_to_Eeprom_slave_board(output_coil, INPUT_OUTPUT_SIZE/2, index_address);
                                      if (write_eeprom_status != WRITE_DONE) {
                                        choose_teach_mode = false; 
                                        Display_eeprom_error(prog_name, index_address);
                                        break;
                                      }
                                      // Reset bit đóng khuôn sau khi đã lưu vào eeprom
                                      if (bitRead(output_coil[FC_VALUE_W1],MOLD_CLOSE)) {output_coil[FC_VALUE_W1] = bitClear(output_coil[FC_VALUE_W1],MOLD_CLOSE);}
                                      // Reset bit lấy sản phẩm sau khi đã lưu vào eeprom
                                      if (bitRead(output_coil[FC_VALUE_W1],GET_PRODUCT)) {output_coil[FC_VALUE_W1] = bitClear(output_coil[FC_VALUE_W1],GET_PRODUCT);}
                                      // Reset bit tới vị trí gắp sau khi đã lưu vào eeprom
                                      //if (bitRead(output_coil[FC_VALUE_W2],ARM_SET_REVERS)) {output_coil[FC_VALUE_W2] = bitClear(output_coil[FC_VALUE_W2],ARM_SET_REVERS);}
                                      // Reset bit tới vị trí thả sau khi đã lưu vào eeprom
                                      //if (bitRead(output_coil[FC_VALUE_W2],ARM_SET_FORWARD)) {output_coil[FC_VALUE_W2] = bitClear(output_coil[FC_VALUE_W2],ARM_SET_FORWARD);}
                                      
                                      index_address += INPUT_OUTPUT_SIZE;
                                      pick_position = false; delay_step = true;
                                      GLCD.text_display_clear(); delay(1);
                                  }break;

          case TEACH_MODE:        // Lưu mã hoàn thành chương trình TEACH_MODE
                                  pick_position = false; delay_step = false; main_position = false; choose_teach_mode = false;
                                  // Lưu mã kết thúc chương trình
                                  write_eeprom_status = Write_single_data_to_Eeprom_slave_board(REPEAT_STEP, index_address);
                                  if (write_eeprom_status != WRITE_DONE) {
                                      choose_teach_mode = false;
                                      Display_eeprom_error(prog_name, index_address);
                                      break;
                                  }
                                  // Lưu mã sẵn sàng chạy auto/modify
                                  write_eeprom_status = Write_single_data_to_Eeprom_slave_board(PROGRAM_READY, Program_start_addr + 2);
                                  if (write_eeprom_status != WRITE_DONE) {
                                      Display_eeprom_error(prog_name, Program_start_addr + 2);
                                      choose_teach_mode = false;
                                      break;
                                  }                          
                                  GLCD.text_display_clear(); delay(1); 
                                  GLCD.printTxt(1,0,"TEACH MODE DONE"); 
                                  GLCD.printTxt(3,0, "Press OP.0/OP.1");
                                  break;                         

          case STOP_01:           // Thoát khỏi chương trình TEACH_MODE và không lưu data
          case STOP_02:           pick_position = false; delay_step = false; main_position = false; choose_teach_mode = false; 
                                  GLCD.text_display_clear(); delay(1); Show_Infor(); break;
        }
        if (choose_teach_mode == false) { break; } // Thoát khỏi vòng lặp while choose_teach_mode
      } // end while pick_position
      if (choose_teach_mode == false) { break; } // Thoát khỏi vòng lặp while choose_teach_mode
    } // end while main_position
    if (choose_teach_mode == false) { break; } // Thoát khỏi vòng lặp while choose_teach_mode
  } // end while choose_teach_mode
} // end if teach_mode
//============================================================================================ 
// Chế độ mô phỏng tay máy chạy step by step chương trình đã teached 
// Modify chương trình: chế độ dùng cảm biến, thời gian delay
//============================================================================================ 
  if (COMMAND == REMOTR_MODE) {
    if (choose_program == true){

      uint16_t index_address;
      uint16_t get_function;
      uint8_t  packet_len;
      uint16_t output_coil[INPUT_OUTPUT_SIZE/2]; 
      
      bool _function = false;  bool modify_program = true;
      bool modify_delay = false; bool modify_vaccum_sensor = false; bool modify_pick_sensor = false;

      output_coil[FC_CODE] = OUTPUT_STEP; output_coil[FC_VALUE_W1] = 0; output_coil[FC_VALUE_W2] = 0;
      GLCD.text_display_clear(); delay(1);
      GLCD.printTxt(0,1,"OP0 "); GLCD.printTxt(0,3,"PROG"); GLCD.print_number(0,5,prog_name); 
      GLCD.printTxt(1,0, "PgDn:GotoNextPg");
      GLCD.printTxt(2,0, "Modify:EditDelay"); 
      GLCD.printTxt(3,0, "Stop:ExitRemotr");
    
      index_address = Program_start_addr + 4; //Nhảy 4 bytes tới mode_program
     
      while(modify_program){
          COMMAND = KEYPAD.Switch_press_scan();
          
          switch(COMMAND) {
            case BUTTON_PAGE_DOWN: 
                  modify_delay = false; modify_vaccum_sensor = false; modify_pick_sensor = false;
                  get_function = Read_single_data_from_Eeprom_slave_board(index_address); 
                  Serial.println(get_function,HEX);
                  for (int i = 0; i < Function_number_max; i++) {
                    if (get_function == u16Function[i]) {_function = true; break;}
                  }
                  if (_function == false) {GLCD.printTxt(3,2,"Error..."); return;}
                  break;
            case MODIFY_MODE:
/*Modify delay value------------------------------------------------------------------------*/
                if (modify_delay) { // Nếu modify ở step delay
                    uint16_t delay_value[STEP_REPEAT_DELAY_SIZE/2]; char delayArray[9]; int8_t count_value;
                    delay_value[FC_CODE] = DELAY_STEP; 
                    index_address -= STEP_REPEAT_DELAY_SIZE; // Quay ngược lại địa chỉ bắt đầu của delay step
                    GLCD.printTxt(2,0,"Modify delay ...");
                    for (int i = 0; i < 9; i++) {delayArray[i] = ' ';} 
                    delayArray[8] = '\0'; GLCD.printTxt(1,3,delayArray); GLCD.printTxt(1,3,"0s"); count_value = 0;
                
                    while(modify_delay) {
                        COMMAND = KEYPAD.Switch_press_scan();
                        switch (COMMAND) {
                          case _1_:    delayArray[count_value++] = '1'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
                          case _2_:    delayArray[count_value++] = '2'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
                          case _3_:    delayArray[count_value++] = '3'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
                          case _4_:    delayArray[count_value++] = '4'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
                          case _5_:    delayArray[count_value++] = '5'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
                          case _6_:    delayArray[count_value++] = '6'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
                          case _7_:    delayArray[count_value++] = '7'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
                          case _8_:    delayArray[count_value++] = '8'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
                          case _9_:    delayArray[count_value++] = '9'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
                          case _0_:    delayArray[count_value++] = '0'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;
                          case _point_:delayArray[count_value++] = '.'; delayArray[count_value] = 's'; delayArray[count_value+1] = '\0';GLCD.printTxt(1,3,delayArray); if (count_value == 7) {count_value = 6;} break;

                          case BUTTON_DEL:  for (int i = 0; i < 9; i++) {delayArray[i] = ' ';} 
                                            delayArray[8] = '\0'; GLCD.printTxt(1,3,delayArray);
                                            GLCD.printTxt(1,3,"0s"); count_value = 0;
                                            break; 
                          case BUTTON_PAGE_DOWN:  delay_value[FC_VALUE_W1] = convert_str_to_delay_number(delayArray);
                                                  uint8_t write_eeprom_status = Write_packet_data_to_Eeprom_slave_board(delay_value, STEP_REPEAT_DELAY_SIZE/2, index_address);
                                                  if (write_eeprom_status != WRITE_DONE) {
                                                    modify_program = false; modify_delay = false;
                                                    Display_eeprom_error(prog_name, index_address);
                                                    break; //Thoat khoi switch
                                                  } delay(100);
                                                  index_address += STEP_REPEAT_DELAY_SIZE;
                                                  get_function = Read_single_data_from_Eeprom_slave_board(index_address); 
                                                  Serial.println(get_function,HEX);
                                                  for (int i = 0; i < Function_number_max; i++) {
                                                    if (get_function == u16Function[i]) {_function = true; break;}
                                                  }
                                                  if (_function == false) {GLCD.printTxt(3,2,"Error.........."); return;}
                                                  modify_delay = false; break;                        
                        }
                    } // while modify_delay
                  } // if modify_delay
/*Modify vaccum sensor------------------------------------------------------------------------*/
                 if (modify_vaccum_sensor){
                    GLCD.printTxt(1,0,"Modify vaccum..."); GLCD.printTxt(2,0,">Use sensor");
                    uint16_t choose_vaccum_sensor[VACCUM_SENSOR_SIZE/2];
                    choose_vaccum_sensor[FC_CODE] = VACCUM_SENSOR; choose_vaccum_sensor[FC_VALUE_W1] = 0;
                    index_address -= VACCUM_SENSOR_SIZE;
                    while (modify_vaccum_sensor){
                      COMMAND = KEYPAD.Switch_press_scan();
                      switch (COMMAND) {
                        case ARM_VACUUM:    
                                      if (!choose_vaccum_sensor[FC_VALUE_W1]){GLCD.printTxt(2,0,">No sensor "); choose_vaccum_sensor[FC_VALUE_W1] = 1;}
                                      else {GLCD.printTxt(2,0,">Use sensor"); choose_vaccum_sensor[FC_VALUE_W1] = 0;}
                                      break;
                        case BUTTON_PAGE_DOWN: 
                                              uint8_t write_eeprom_status = Write_packet_data_to_Eeprom_slave_board(choose_vaccum_sensor, VACCUM_SENSOR_SIZE/2, index_address);
                                              if (write_eeprom_status != WRITE_DONE) {
                                                modify_program = false; modify_vaccum_sensor = false;
                                                Display_eeprom_error(prog_name, index_address);
                                                break;
                                              } delay(100);
                                              index_address += VACCUM_SENSOR_SIZE;
                                              get_function = Read_single_data_from_Eeprom_slave_board(index_address); 
                                              Serial.println(get_function,HEX);
                                              for (int i = 0; i < Function_number_max; i++) {
                                                if (get_function == u16Function[i]) {_function = true; break;}
                                              }
                                              if (_function == false) {GLCD.printTxt(3,2,"Error.........."); return;}
                                              modify_vaccum_sensor = false; break;
                                        }
                    } // while modify_vaccum_sensor
                 } // if modify_vaccum_sensor
/*Modify pick sensor------------------------------------------------------------------------*/
                 if (modify_pick_sensor){
                   GLCD.printTxt(1,0,"Modify grip ..."); GLCD.printTxt(2,0,">Use sensor"); 
                   uint16_t choose_pick_sensor[VACCUM_SENSOR_SIZE/2];
                   choose_pick_sensor[FC_CODE] = PICK_SENSOR; choose_pick_sensor[FC_VALUE_W1] = 0;
                   index_address -= VACCUM_SENSOR_SIZE;
                   while (modify_pick_sensor){
                      COMMAND = KEYPAD.Switch_press_scan();
                      switch (COMMAND) {
                        case ARM_VACUUM:    
                                      if (!choose_pick_sensor[FC_VALUE_W1]){GLCD.printTxt(2,0,">No sensor "); choose_pick_sensor[FC_VALUE_W1] = 1;}
                                      else {GLCD.printTxt(2,0,">Use sensor"); choose_pick_sensor[FC_VALUE_W1] = 0;}
                                      break;
                        case BUTTON_PAGE_DOWN: 
                                              uint8_t write_eeprom_status = Write_packet_data_to_Eeprom_slave_board(choose_pick_sensor, VACCUM_SENSOR_SIZE/2, index_address);
                                              if (write_eeprom_status != WRITE_DONE) {
                                                modify_program = false; modify_pick_sensor = false;
                                                Display_eeprom_error(prog_name, index_address);
                                                break;
                                              } delay(100);
                                              index_address += VACCUM_SENSOR_SIZE;
                                              get_function = Read_single_data_from_Eeprom_slave_board(index_address); 
                                              Serial.println(get_function,HEX);
                                              for (int i = 0; i < Function_number_max; i++) {
                                                if (get_function == u16Function[i]) {_function = true; break;}
                                              }
                                              if (_function == false) {GLCD.printTxt(3,2,"Error.........."); return;}
                                              modify_pick_sensor = false; break;
                                        }
                   } // while modify_pick_sensor
                 } // if modify_pick_sensor
                 break;
            case STOP_01: 
            case STOP_02:
                  modify_program = false; 
                  GLCD.text_display_clear(); delay(1);
                  Show_Infor();
                  _function = false;
                  break;
          }
/*Thực hiện monitor------------------------------------------------------------------------*/
          while (_function) {
          Serial.println(get_function, HEX);
/* Hiện thị mã chọn mode tay máy*/
            if (get_function == MODE_ARM){

                GLCD.text_display_clear(); delay(1);
                GLCD.printTxt(0,0,"N00:MODE"); GLCD.printTxt(0,4," PROG"); GLCD.print_number(0,6,prog_name);
                packet_len = MODE_ARM_SIZE/2; // 2 words
                uint16_t *mode_function = Read_packet_data_from_Eeprom_slave_board(packet_len, index_address);
                switch (mode_function[FC_VALUE_W1]){
                    case MAIN_ARM_MODE: GLCD.printTxt(1,0,">Mode tay hut"); break;
                    case PICK_ARM_MODE: GLCD.printTxt(1,0,">Mode tay gap"); break;
                    case TWO_ARM_MODE:  GLCD.printTxt(1,0,">Mode ket hop"); break;
                }
                GLCD.printTxt(3,0," Press PgDn ...");
                index_address += MODE_ARM_SIZE; _function = false;
                break;}
/* Hiện thị mã chọn cảm biến chân không */                   
            if (get_function == VACCUM_SENSOR) {

                GLCD.text_display_clear(); delay(1);
                GLCD.printTxt(0,0,"N01:USE VACCUM"); GLCD.printTxt(1,0,"Vaccum sensor: ");
                packet_len = VACCUM_SENSOR_SIZE/2; // 2 words
                uint16_t *vaccum_function = Read_packet_data_from_Eeprom_slave_board(packet_len, index_address);
                switch (vaccum_function[FC_VALUE_W1]){
                    case 0:  GLCD.printTxt(2,0,">Use sensor"); break;
                    case 1:  GLCD.printTxt(2,0,">No sensor"); break;
                }
                GLCD.printTxt(3,0," Pr.PgDn/Modify");
                index_address += VACCUM_SENSOR_SIZE; _function = false; modify_vaccum_sensor = true;
                break;}
/* Hiện thị mã chọn cảm biến tay kẹp */
            if (get_function == PICK_SENSOR) {

                GLCD.text_display_clear(); delay(1);
                GLCD.printTxt(0,0,"N02:USE GRIP"); GLCD.printTxt(1,0,"Grip sensor: ");
                packet_len = VACCUM_SENSOR_SIZE/2; // 2 words
                uint16_t *pick_function = Read_packet_data_from_Eeprom_slave_board(packet_len, index_address);
                switch (pick_function[FC_VALUE_W1]){
                    case 0:  GLCD.printTxt(2,0,">Use sensor"); break;
                    case 1:  GLCD.printTxt(2,0,">No sensor"); break;
                }

                GLCD.printTxt(3,0," Pr.PgDn/Modify");
                index_address += VACCUM_SENSOR_SIZE; _function = false; modify_pick_sensor = true;
                break;}
/* Vị trí chờ lấy sản phẩm */
            if (get_function == WAITING_POSSITON){

                GLCD.text_display_clear(); delay(1);
                GLCD.printTxt(0,0,"N03:WAITING POS");
                packet_len = INPUT_OUTPUT_SIZE/2; // 3 words
                uint16_t *waiting_function = Read_packet_data_from_Eeprom_slave_board(packet_len, index_address);
                if(bitRead(waiting_function[FC_VALUE_W2],ARM_SET_ROTATE_UP)){GLCD.printTxt(1,0,">Tay xoay len  ");}
                if(bitRead(waiting_function[FC_VALUE_W2],ARM_SET_ROTATE_DOWN)){GLCD.printTxt(1,0,">Tay xoay xuong");} 
                if(bitRead(waiting_function[FC_VALUE_W2],ARM_SUB_LEFT_RIGHT)){GLCD.printTxt(2,0,">Tay di toi ");} 
                else GLCD.printTxt(2,0,">Tay di lui ");
                /*Xuất giá trị output*/
                output_coil[FC_VALUE_W1] = waiting_function[FC_VALUE_W1]; 
                output_coil[FC_VALUE_W2] = waiting_function[FC_VALUE_W2];
                Write_packet_data_to_output_slave_board(output_coil, INPUT_OUTPUT_SIZE/2); // xuất output_coil cho tay máy chạy
                GLCD.printTxt(3,0," Press PgDn ...");
                index_address += INPUT_OUTPUT_SIZE; _function = false;
                break;}
/* Thời gian delay */
            if (get_function == DELAY_STEP){

                GLCD.text_display_clear(); delay(1);
                GLCD.printTxt(0,0,"N04:DELAY POS"); GLCD.printTxt(1,0,"Delay: "); 
                packet_len = STEP_REPEAT_DELAY_SIZE/2; //2 words
                uint16_t *delay_function = Read_packet_data_from_Eeprom_slave_board(packet_len, index_address);
                char delayvalue[9];
                char *delayArray = convert_delay_number_to_str(delay_function[FC_VALUE_W1]);
                /* hiển thị giá trị delayvalue lên GLCD */
                for (int i = 0; i < 9; i++) {delayvalue[i] = delayArray[i]; if (delayArray[i] == '\0') { break;}} 
                                            GLCD.printTxt(1,3,delayvalue);
                GLCD.printTxt(3,0," Pr.PgDn/Modify");
                index_address += STEP_REPEAT_DELAY_SIZE; _function = false; modify_delay = true; // Đang ở function delay step
                break;}
/* Output coil */
            if (get_function == OUTPUT_STEP){

                GLCD.text_display_clear(); delay(1);
                GLCD.printTxt(0,0,"N05:DEFINE POS"); GLCD.printTxt(1,0,"Action: ...");
                packet_len = INPUT_OUTPUT_SIZE/2; // 3 words
                uint16_t *output_function = Read_packet_data_from_Eeprom_slave_board(packet_len, index_address);
                // Kiểm tra giá trị output_function có bị lỗi không
                if ((output_function[FC_VALUE_W1] == 0xFFFF) || (output_function[FC_VALUE_W2] == 0xFFFF)) 
                { GLCD.printTxt(3,0,"Eeprom error..."); break; }
                /*Giám sát giá trị đọc được từ eeprom */
                Serial.println(output_function[FC_VALUE_W1],HEX);
                Serial.println(output_function[FC_VALUE_W2],HEX);
                Check_status_of_output_coil_word(output_function[FC_VALUE_W2]);
                GLCD.printTxt(3,0," Press PgDn ...");
                /*Xuất giá trị output*/
                output_coil[FC_VALUE_W1] = output_function[FC_VALUE_W1]; 
                output_coil[FC_VALUE_W2] = output_function[FC_VALUE_W2];
                Write_packet_data_to_output_slave_board(output_coil, INPUT_OUTPUT_SIZE/2); // xuất output_coil cho tay máy chạy
                index_address += INPUT_OUTPUT_SIZE; _function = false;
                break;}
/* mã kết thúc 1 chu kỳ chương trình */             
            if (get_function == REPEAT_STEP){

                GLCD.text_display_clear(); delay(1);
                GLCD.printTxt(1,1,"END PROGRAM"); 
                GLCD.printTxt(3,0, "Press Stop..."); _function = false;
                break;}

          } // end while _function
      } // end while _program  
      choose_program = false;
    } 
    else delay(1);
  } // end modify_program
//============================================================================================ 
// Ghi PROGRAM_NOT_READY vào eeprom board slave để cho phép vào được teach mode
//============================================================================================ 
  if (COMMAND == BUTTON_DEL){
    if (choose_program == true){

      uint16_t index_address; uint8_t data_word_length = 10;
      uint16_t data_delete[data_word_length];
      uint8_t write_eeprom_status_01, write_eeprom_status_02;
      for (int i = 0; i < data_word_length; i++) {data_delete[i] = 0xFFFF;}
      index_address = Program_start_addr + 2;

      // Ghi mã PROGRAM_NOT_READY vào địa chỉ index_address epprom board slave
      write_eeprom_status_01 = Write_single_data_to_Eeprom_slave_board (PROGRAM_NOT_READY, index_address);
      // Ghi 10 word FFFF vào cách thanh ghi tiếp theo
      write_eeprom_status_02 = Write_packet_data_to_Eeprom_slave_board(data_delete, data_word_length, index_address + 2); 
      if ((write_eeprom_status_01 != WRITE_DONE)) {
          Display_eeprom_error(prog_name, index_address);
      }
      else {
      GLCD.text_display_clear(); delay(1);
      GLCD.printTxt(1,1,"DELETED DONE"); 
      GLCD.printTxt(3,0, "Press OP.0/OP.1");
      choose_program = false;  
      }
    }
    else delay(1);
  }
//============================================================================================ 
// Send command run auto to board slaver
//============================================================================================ 
  if (COMMAND == AUTO_MODE_01 || COMMAND == AUTO_MODE_02) { 
    if (choose_program == true) { 
      NODE.writeSingleCoil(AUTO_RUN_MODBUS_ADDR, CHOOSE);

      GLCD.text_display_clear(); delay(1);
      GLCD.printTxt(0,1,"OP0 "); GLCD.printTxt(0,3,"PROG"); GLCD.print_number(0,5,prog_name); 
      GLCD.printTxt(1,0, "AUTO RUNING");
      GLCD.printTxt(2,0, "Monitor Mode"); 
      GLCD.printTxt(3,0, "Stop:ExitRun");

      COMMAND = KEYPAD.Switch_press_scan();
      
      switch (COMMAND){
        case MONITOR_MODE: break;

        case STOP_01:
        case STOP_02: break;
      }
      choose_program = false;
    }
    else delay(1);  
  }
//============================================================================================ 
// Dừng tay máy tại chỗ
//============================================================================================ 
  if (COMMAND == STOP_01 || COMMAND == STOP_02) {
    GLCD.text_display_clear(); delay(1);
    GLCD.printTxt(0,1,"STOP ROBOT"); delay(1);
    NODE.writeSingleCoil(HOME_MODBUS_ADDR, CHOOSE);
    delay(100);
    NODE.writeSingleCoil(HOME_MODBUS_ADDR, NOCHOOSE);
    COMMAND = 0xFFFF;
  }
//============================================================================================ 
// Chế độ monitor trong quá trình tay máy đang làm việc
//============================================================================================ 
  if (COMMAND == MONITOR_MODE){
    if (choose_program == true || choose_teach_mode == true) { 
      COMMAND = 0xFFFF;
        
        uint16_t *receive_data;
        receive_data = Read_packet_data_from_Eeprom_slave_board(20, Program_start_addr);
        Serial.println("Gia tri receive_data 01: "); 
        for (int i = 0; i < 20; i++) {
            Serial.print(receive_data[i],HEX); Serial.print("_");
        } Serial.println("");
        
        receive_data = Read_packet_data_from_Eeprom_slave_board(20, Program_start_addr+40);
        Serial.println("Gia tri receive_data 02: "); 
        for (int i = 0; i < 20; i++) {
            Serial.print(receive_data[i],HEX); Serial.print("_");
        } Serial.println("");
        
        uint8_t count_monitor = 0; 
        uint16_t *read_packet_monitor;
        GLCD.text_display_clear(); delay(1);
        GLCD.printTxt(0,1,"MONITOR_MODE"); delay(1);
/*Vào vòng lặp monitor, cứ 100 ms đọc giá trị 1 lần*/
        while(COMMAND != MONITOR_MODE) {
            COMMAND = KEYPAD.Scan_Keypad_5x7();
            read_packet_monitor = Read_packet_data_from_feedback_register_slave_board(FEEDBACK_SIZE);
            Serial.println("Gia tri receive_data: ");
            for (int i = 0; i < FEEDBACK_SIZE; i++) {
                Serial.print(read_packet_monitor[i],HEX); Serial.print("_");
            }
            if (read_packet_monitor[FC_CODE] == MODE_ARM){

            }
            if (read_packet_monitor[FC_CODE] == VACCUM_SENSOR){

            }
            if (read_packet_monitor[FC_CODE] == PICK_SENSOR){

            }           
            if (read_packet_monitor[FC_CODE] == WAITING_POSSITON){

            }
            if (read_packet_monitor[FC_CODE] == OUTPUT_STEP){

            }     
            if (read_packet_monitor[FC_CODE] == DELAY_STEP){

            }  
            if (read_packet_monitor[FC_CODE] == REPEAT_STEP){

            }                             
            count_monitor++; if (count_monitor == 10) {count_monitor = 0;}
            GLCD.print_number(1,3,Number_STT[count_monitor]);
            delay(200);
        }
        choose_program = false; choose_teach_mode = false;
        GLCD.text_display_clear(); delay(1);
        Show_Infor();    
    } else delay(1);
    COMMAND = 0xFFFF;
  }
//============================================================================================
  if (COMMAND == MANUAL_MODE) {
    // Vào chế độ Manual, điều khiển các Output của máy
      bool COMMAND_MANUAL_MODE = true;
      uint16_t output_coil[INPUT_OUTPUT_SIZE/2]; output_coil[FC_CODE] = OUTPUT_STEP;
      uint16_t output_coil_index[INPUT_OUTPUT_SIZE/2];

      output_coil[FC_VALUE_W1] = 0;
      output_coil[FC_VALUE_W2] = 0;
      output_coil_index[FC_VALUE_W1] = output_coil[FC_VALUE_W1];
      output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2];
      
      GLCD.text_display_clear(); delay(1);
      GLCD.printTxt(0,1,"MANUAL_MODE");
      GLCD.printTxt(1,0,"Action:"); 
      GLCD.printTxt(3,1, "Press Enter");
      while (COMMAND_MANUAL_MODE) {
        COMMAND_MANUAL = KEYPAD.Switch_press_scan();
        switch (COMMAND_MANUAL){
          case ARM_ROTATE_UP:     
            output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2];
            output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_SET_ROTATE_UP);  
            output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_SET_ROTATE_DOWN);
            GLCD.printTxt(2,0,">Tay xoay len  "); break;

          case ARM_ROTATE_DOWN:   
            output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2];
            output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_SET_ROTATE_DOWN);
            output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_SET_ROTATE_UP);                                 
            GLCD.printTxt(2,0,">Tay xoay xuong"); break;

          case ARM_GO_LEFT:       
            output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2]; 
            output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_SUB_LEFT_RIGHT);                                  
            GLCD.printTxt(2,0,">Tay di toi    "); break;

          case ARM_GO_RIGHT:      
            output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2]; 
            output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_SUB_LEFT_RIGHT);                                 
            GLCD.printTxt(2,0,">Tay di lui    "); break;

          case ARM_UP:            
            output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2]; 
            output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_SUB_UP_DOWN);                                 
            GLCD.printTxt(2,0,">Tay di len    "); break;

          case ARM_DOWN:          
            output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2];
            output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_SUB_UP_DOWN);                                
            GLCD.printTxt(2,0,">Tay di xuong  "); break;

          case ARM_FORWARD:       
            output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2]; 
            output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_SET_FORWARD);
            output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_SET_REVERS);                                 
            GLCD.printTxt(2,0,">Toi vi tri tha"); break;

          case ARM_REVERS:        
            output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2];
            output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_SET_REVERS);
            output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_SET_FORWARD);                                 
            GLCD.printTxt(2,0,">Toi vi tri gap"); break;

          case ARM_PICK:          
            output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2]; 
            output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_PICK_UNPICK_01);
            output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_PICK_UNPICK_02);
            output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_SET_VACUUM_01);
            output_coil_index[FC_VALUE_W2] = bitSet(output_coil_index[FC_VALUE_W2],ARM_SET_VACUUM_02);                                 
            GLCD.printTxt(2,0,">Lay san pham  "); break;

          case ARM_UNPICK:        
            output_coil_index[FC_VALUE_W2] = output_coil[FC_VALUE_W2]; 
            output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_PICK_UNPICK_01);
            output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_PICK_UNPICK_02);
            output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_SET_VACUUM_01);
            output_coil_index[FC_VALUE_W2] = bitClear(output_coil_index[FC_VALUE_W2],ARM_SET_VACUUM_02);                                 
            GLCD.printTxt(2,0,">Tha san pham  "); break;

          case BUTTON_ENTER:      
            if ((output_coil[FC_VALUE_W2] != output_coil_index[FC_VALUE_W2]) || (output_coil[FC_VALUE_W1] != output_coil_index[FC_VALUE_W1])){
              output_coil[FC_VALUE_W1] = output_coil_index[FC_VALUE_W1]; 
              output_coil[FC_VALUE_W2] = output_coil_index[FC_VALUE_W2];
              Check_status_of_output_coil_word(output_coil[FC_VALUE_W2]);  
              Write_packet_data_to_output_slave_board(output_coil, INPUT_OUTPUT_SIZE/2); // xuất output_coil cho tay máy chạy
            } break; 
            
          case MANUAL_MODE:  // Exit the manual mode
            COMMAND_MANUAL_MODE = false;
            GLCD.text_display_clear(); delay(1); Show_Infor(); break;
        } // end switch
      } // end while COMMAND_MANUAL_MODE
  }
//================================================================================================================
// Gửi command tới board slaver đưa tay máy về vị trí HOME (vị trí gắp sản phẩm)
//================================================================================================================
  if (COMMAND == HOME) {
    GLCD.text_display_clear(); delay(1);
    GLCD.printTxt(0,1,"HOME_MODE"); delay(1);
    NODE.writeSingleCoil(HOME_MODBUS_ADDR, CHOOSE);
    delay(100);
    NODE.writeSingleCoil(HOME_MODBUS_ADDR, NOCHOOSE);
    COMMAND = 0xFFFF;
  }
//============================================================================================ 
//============================================================================================
  Serial.println ("Main loop");
  delay(1);
}// END LOOP
//================================================================================================================
// Hiện thị epprom error ở chương trình program_name
void Display_eeprom_error(uint16_t program_name, uint16_t addr){
  Serial.print("Eeprom bị lỗi bắt đầu ở địa chỉ: "); Serial.println(addr,HEX);
  GLCD.text_display_clear(); delay(1);
  GLCD.printTxt(0,0,"OP0 "); GLCD.printTxt(0,2,"PROG"); GLCD.print_number(0,4,program_name);
  GLCD.printTxt(2,1,"EEPROM ERROR");
}
//================================================================================================================
// Chuyển đổi chuỗi thành số delay
//================================================================================================================
uint16_t convert_str_to_delay_number(char* stringArr)
{
  int myflag = 0; float s = 0; float v = 0;
  int i = 1; int sign = 1;
  float result = 0;

  for(int m = 0; m < strlen(stringArr) + 1; m++)
    {
      char ch = stringArr[m];
      switch(ch)
      {
       case '0'...'9':
                if (myflag == 0)            // tach phan nguyen
                  {
                    v = v*10 + ch - '0';    // đổi kiểu char thành kiểu số
                  }
                if (myflag == 1)            // tach phan thap phan
                  {
                    s = ch - '0';
                    v = v + s*(pow(10,-i));
                    i += 1;
                  }
                break;
       case '\0': 
                result = v*sign;            // Toa do Y
                break;
       case '.':
                myflag = 1;
                break;
       case '-':
                sign = -1;
                break;          
      }
    }
    result = result*1000; 
    if (result > 9000) {result = 9000;} // Giới hạn giá trị delay tối đa là 9 giây
    return result;
 }
//================================================================================================================
// Chuyển đổi số thực thành chuỗi delay: 1.250 -> "1.250"
//================================================================================================================
char *convert_delay_number_to_str(uint16_t value){
    static char result[8];
    float delay_value = value/1000; // lấy giá trị số thực delay_value cần hiển thị

    result[0] = value/1000%10 + '0';
    result[1] = '.';
    result[2] = value/100%10 + '0';
    result[3] = value/10%10 + '0';
    result[4] = value%10 + '0';
    result[5] = 's';
    result[6] = '\0';

    return result;
}
//================================================================================================================
// Kiểm tra trạng thái các bit trong thanh ghi output coil
// In ra GLCD mã HEX output_coil[FC_VALUE_W2]
//================================================================================================================
void Check_status_of_output_coil_word(uint16_t data){
    uint8_t HighData = highByte(data); uint8_t code[4];
    uint8_t LowData = lowByte(data); float num_index = 0;
    float num0,num1,num2,num3;
    num0 = num1 = num2 = num3 = 0;

    for (int i = 4; i < 8; i++){
      float bit = bitRead(HighData,i); float number = pow(2,i-4); 
      num_index =  bit*number; num0 += num_index; // code = 0 ... F
    } int code0 = round(num0); code[0] = Number_STT[code0];
      
    for (int j = 0; j < 4; j++) {
      float bit = bitRead(HighData,j); float number = pow(2,j); 
      num_index =  bit*number; num1 += num_index; // code = 0 ... F   
    } int code1 = round(num1); code[1] = Number_STT[code1];

    for (int k = 4; k < 8; k++){
      float bit = bitRead(LowData,k); float number = pow(2,k-4); 
      num_index =  bit*number; num2 += num_index; // code = 0 ... F
    } int code2 = round(num2); code[2] = Number_STT[code2];

    for (int f = 0; f < 4; f++) {
      float bit = bitRead(LowData,f); float number = pow(2,f); 
      num_index =  bit*number; num3 += num_index; // code = 0 ... F   
    } int code3 = round(num3); code[3] = Number_STT[code3]; 

    GLCD.print_u16code(1,4,code);
}