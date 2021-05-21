#include "ModbusSlave.h"
#include "EEPROM_25LC256.h"
#include "EEPROM_24C256.h"
#include "StepperMotor.h"
#include "MyMCP23017.h"
#include "My74HC595.h"
#include "MyLS7366.h"
#include "TAYMAY.h"
//================================================================
#define CB_DONG_KHUON_HET               23   // Input cảm biến đóng khuôn hết
#define CB_MO_KHUON_HET                 25   // Input cảm biến mở khuôn hết
#define CLOSE_MOLD_COUNTER              3750  // hard code thời gian đóng khuôn trong ngắt timer2
//================================================================
#define DONG_KHUON                  24   // Chan kich dong khuon
#define CHE_DO_ROBOT                26   // Chan kich che do robot
#define COI_BAO                     28   // Còi báo
#define RESET_PIN                   30   // Kích reset
//================================================================
#define ARDUINO_ADDRESS             2    // Dia chi board arduino slaver can dieu khien
//================================================================
#define MCP23017_I2C_ADDRESS_01        0x40 // MCP23017_I2C_ADDRESS 0x20 + 0 = 0x40
#define MCP23017_I2C_ADDRESS_02        0x50
#define MCP23017_I2C_ADDRESS_03        0x60
#define MCP23017_I2C_ADDRESS_04        0x70
// 1 0 1 0  0 A1 A0 R/W
#define ADDR_EEPROM_24C256_01          0xA0      // 1 0 1 0  0 0 0 R/W
#define ADDR_EEPROM_24C256_02          0xA2      // 1 0 1 0  0 0 1 R/W
#define SELECT_EEPROM_25LC256_01       53
#define SELECT_EEPROM_25LC256_02       54
//================================================================
// Khai báo các chân chip select LS7366 read encoder
#define SELECT_LS7366_X            33
#define SELECT_LS7366_Y            35
#define SELECT_LS7366_Z            37
#define SELECT_LS7366_A            39
#define SELECT_LS7366_AC           39
//================================================================
// Khai báo các chân xung chiều chạy động cơ stepper/servo
#define PWM_MOTOR_01                11
#define DIR_MOTOR_01                12
#define PWM_MOTOR_02                3
#define DIR_MOTOR_02                5
#define PWM_MOTOR_03                6   
#define DIR_MOTOR_03                7
#define PWM_MOTOR_04                44
#define DIR_MOTOR_04                46
//================================================================
// Khai báo các chân phục vụ xuất data output
#define SR_DATA_PIN                 32
#define SR_CLK_PIN                  34
#define OE_PIN                      36
#define LATCH_PIN                   38
#define RST_PIN                     40

Modbus                  node_slave(MODBUS_SERIAL, ARDUINO_ADDRESS, MODBUS_CONTROL_PIN);
EEPROM_25LC256          EEPROM256_01 ,EEPROM256_02;        
EEPROM_24C256           eeprom_type, EEPROM24C256_01, EEPROM24C256_02;          
StepperMotor            Motor_X, Motor_Y, Motor_Z, Motor_A;
MyMCP23017              MCP23017_01, MCP23017_02;
My74HC595               Output_port;
MyLS7366                LS7366_Motor_X, LS7366_Motor_Y, LS7366_Motor_Z, LS7366_Motor_A, LS7366_Motor_AC;

bool coil[16]; uint8_t coil_size = sizeof(coil) / sizeof(coil[0]); // Khai bao coil 
uint16_t state_coil;              // Gia tri tuong ung voi coil[i]
uint16_t program_start_addr;      // Địa chỉ bắt đầu chạy khi nhấn nút Auto
uint16_t choose_eeprom_type;      // Chọn eeprom cần đọc
uint8_t feedback_data[MAX_FARME_DATASIZE]; // lưu giá trị để truyền về board master trong chế độ run/monitor
uint8_t feedback_data_size; bool ready_send = false;

static uint32_t pulse_set_X, pulse_set_Y, pulse_set_Z, pulse_set_A;
static uint32_t pulse_set_AC = 3200; // Số xung cài đặt trước
bool pulse_done_X, pulse_done_Y, pulse_done_Z, pulse_done_A, pulse_done_AC;
uint8_t output_coil[INPUT_OUTPUT_SIZE -2]; // Lưu giá trị output coil nhận từ board master
uint8_t intput_value[INPUT_OUTPUT_SIZE-2]; //giá trị input value được tạo ra tương ứng với ouput_coil
bool robot_mode = false; bool home_position = false;
bool auto_running = false; 
bool vaccum_sensor = false; bool pick_sensor = false;  // biến lựa chọn cảm biến chân không và tay kẹp
bool main_arm_mode = false; bool pick_arm_mode = false; bool two_arm_mode = false; // Biến lựa chọn mode arm
bool AC_3phase_motor_run = false;
bool Ac_motor_trans_speed = false; // Thay đổi tốc độ động cơ Ac 
bool MODE_DONG_KHUON = false;  // Vào chế độ đóng khuôn trong ngắt timer
bool MO_KHUON_HET = false;     // Trạng thái khuôn mở hết, sẵn sàng cho tay máy lấy sản phẩm
bool MODE_GET_PRODUCT = false; // Khai báo vào bước lấy sản phẩm
bool exceed_pulse = false;
uint16_t address_run_auto;


uint8_t u8receive_buffer_data[MAX_FARME_DATASIZE]; // Bộ nhớ đệm lưu packet_data nhận từ master
uint16_t u16receive_buffer_data[MAX_FARME_DATASIZE];
uint16_t command_modbus = 0xFFFF;                // Lưu lệnh cần ghi với eeprom
uint8_t  length_write_to_eeprom = 0;             // Số byte cần ghi vào eeprom
uint16_t  address_write_to_eeprom = 0;           // Địa chỉ eeprom đầu tiên cần ghi
//===============================================================================================================
// Do chọn thanh ghi lưu giá trị so sánh với TCNTn trong thanh ghi ICRn trong phần begin(), cho nên phải chọn ngắt tràn ICRn
//===============================================================================================================
ISR (TIMER1_CAPT_vect) // Ngat tran ICR1, ICF1 = 1
{
  static uint32_t pulse_count_motor_X; pulse_count_motor_X ++;
  if (pulse_count_motor_X == pulse_set_X)
  {
    Motor_X.detach();
    pulse_count_motor_X = 0;
    pulse_done_X = true;
    uint32_t encoder_counter_X = LS7366_Motor_X.read_counter_register();
    uint8_t  encoder_status_X  = LS7366_Motor_X.read_status_register();
    encoder_stop_count(LS7366_Motor_X);
  }
}
ISR (TIMER3_CAPT_vect) // Ngat tran ICR3, ICF3 = 1
{
  static uint32_t pulse_count_motor_Y; pulse_count_motor_Y ++;
  if (pulse_count_motor_Y == pulse_set_Y)
  {
    Motor_Y.detach();
    pulse_count_motor_Y = 0;
    pulse_done_Y = true;
    uint32_t encoder_counter_Y = LS7366_Motor_Y.read_counter_register();
    uint8_t  encoder_status_Y  = LS7366_Motor_Y.read_status_register();
    encoder_stop_count(LS7366_Motor_Y);
  }
}
ISR (TIMER4_CAPT_vect) // Ngat tran ICR4, ICF4 = 1
{
  static uint32_t pulse_count_motor_Z; pulse_count_motor_Z ++;
  if (pulse_count_motor_Z == pulse_set_Z)
  {
    Motor_Z.detach();
    pulse_count_motor_Z = 0;
    pulse_done_Z = true;
    uint32_t encoder_counter_Z = LS7366_Motor_Z.read_counter_register();
    uint8_t  encoder_status_Z  = LS7366_Motor_Z.read_status_register();
    encoder_stop_count(LS7366_Motor_Z);
  }
}
ISR (TIMER5_CAPT_vect) // Ngat tran ICR5, ICF5 = 1
{
  static uint32_t pulse_count_motor_A; pulse_count_motor_A ++;
  if (pulse_count_motor_A == pulse_set_A)
  {
    Motor_A.detach();
    pulse_count_motor_A = 0;
    pulse_done_A = true;
    uint32_t encoder_counter_A = LS7366_Motor_A.read_counter_register();
    uint8_t  encoder_status_A  = LS7366_Motor_A.read_status_register();
    encoder_stop_count(LS7366_Motor_A);
  }
}
//================================================================
//================================================================
void encoder_stop_count(MyLS7366 &LS7366){
    LS7366.write_mode_register_1(NO_FLAGS | DIS_CNTR | BYTE_4);
    LS7366.clear_counter_register(); LS7366.clear_status_register();
}
//================================================================
void encoder_start_count(MyLS7366 &LS7366){
    LS7366.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_4);
}
//================================================================
// Đọc giá trị input từ 2 MCP23017: return: FFFF FFFF 
// Hàm đọc giá trị cảm biến được đặt trong hàm ngắt timer2 để đọc liên tục
uint8_t* Read_Input_From_MCP23017(void){
    static uint8_t data_input[INPUT_OUTPUT_SIZE - 2];

    data_input[0] = MCP23017_01.Read_MCP23017(MCP23017_GPIOA);
    data_input[1] = MCP23017_01.Read_MCP23017(MCP23017_GPIOB);
    data_input[2] = MCP23017_02.Read_MCP23017(MCP23017_GPIOA);
    data_input[3] = MCP23017_02.Read_MCP23017(MCP23017_GPIOB);

    return data_input;
}
//================================================================
// Đọc data từ EEPROM256 khi gặp mã nhận diện: FFFF FFFF FFFF FFFF FFFF FFFF FFFF FFFF FFFF
// Trả về mảng index byte giá trị
uint8_t* Read_Program_Function_From_EEPROM (EEPROM_24C256 &EEPROM256, uint8_t index, uint16_t address){
  static uint8_t data_index[MAX_FARME_DATASIZE]; // Khai báo static uint8_t *data_index bị lỗi lưu data ????
  uint8_t result = 1;  // index: số byte cần đọc từ EEPROM256

  result = EEPROM256.Read_Mutiple_data_from_EEPROM (address, index);  // Địa chỉ và số byte cần đọc từ EEPROM256
  if (result == 0){ 
    for (int i = 0; i < index; i++) { 
      data_index[i] = EEPROM256.getResponseBuffer(i); 
    } 
  }
  return data_index;
}
//================================================================
// Ghi data vao EEPROM256 theo Functions, nhận từ master
void Write_Program_Function_to_EEPROM (EEPROM_24C256 &EEPROM256, uint8_t index, uint16_t address, uint8_t *data){
  uint8_t result;
  // Ghi mảng data số lượng index byte bắt đầu ở địa chỉ address của EEPROM256
  result = EEPROM256.Write_Mutiple_data_to_EEPROM(data, address, index);
}
//================================================================
// Read 2 byte data from EEPROM để đọc mã nhận diện
uint16_t Read_2byte_data_from_EEPROM(EEPROM_24C256 &EEPROM256, uint8_t address){
  uint8_t data[2];
  data[0] = EEPROM256.Read_from_EEPROM(address);
  data[1] = EEPROM256.Read_from_EEPROM(address+1);

  return UInt16_word(data,0);
}
//================================================================
// Write 2 byte data from EEPROM để ghi số thứ tự (tên) chương trình
void Write_2byte_data_to_EEPROM(EEPROM_24C256 &EEPROM256, uint16_t data, uint16_t address){
  EEPROM256.Write_to_EEPROM(address, data >> 8);
  EEPROM256.Write_to_EEPROM(address + 1, data);  
}
//================================================================
// Thực hiện lệnh DELAY_STEP
void Execute_DelayStep(uint16_t delay_value){
    if (delay_value > 0){delay(delay_value);}
    else delay(1);
}
//================================================================
// Thực hiện lệnh phát xung chạy trục X_AXIS
void Execute_XaxisStep(int32_t Xaxis_value, uint32_t freq){
  pulse_done_X = false;
  pulse_set_X = abs(Xaxis_value);
  encoder_start_count(LS7366_Motor_X);
  Motor_X.generate_pwm(Xaxis_value, freq);
}
//================================================================
// Thực hiện lệnh phát xung chạy trục Y_AXIS
void Execute_YaxisStep(int32_t Yaxis_value, uint32_t freq){
  pulse_done_Y = false;
  pulse_set_Y = abs(Yaxis_value);
  encoder_start_count(LS7366_Motor_Y);
  Motor_Y.generate_pwm(Yaxis_value, freq);
}
//================================================================
// Thực hiện lệnh phát xung chạy trục Z_AXIS
void Execute_ZaxisStep(int32_t Zaxis_value, uint32_t freq){
  pulse_done_Z = false;
  pulse_set_Z = abs(Zaxis_value);
  encoder_start_count(LS7366_Motor_Z);
  Motor_Z.generate_pwm(Zaxis_value, freq);
}
//================================================================
// Thực hiện lệnh phát xung chạy trục A_AXIS
void Execute_AaxisStep(int32_t Aaxis_value, uint32_t freq){
  pulse_done_A = false;
  pulse_set_A = abs(Aaxis_value);
  encoder_start_count(LS7366_Motor_A);
  Motor_A.generate_pwm(Aaxis_value, freq);
}
//================================================================
// Thực hiện lệnh chạy trục X dùng AC motor
void Execute_X_ACmotor(int32_t Xaxis_value, uint8_t command){

}
//================================================================
// Thực hiện lệnh OUTPUT_STEP
// value: giá trị cần xuất ra ngõ output
// wQty: số lượng byte cần xuất ra ngõ output
void Execute_OutputStep(uint8_t *value, uint8_t wQty){
  uint8_t output_value[INPUT_OUTPUT_SIZE-2]; //giá trị xuất ra ngõ output
  uint16_t output_value_index[2];

  output_value_index[FC_VALUE_W1] = UInt16_word(value,0);  // output điều khiển khuôn và thực thi chạy động cơ AC
  output_value_index[FC_VALUE_W2] = UInt16_word(value,2);  // output điều khiển khí nén và command động cơ AC
/* Kiểm tra chế độ robot kết nối với máy ép nhựa 
 Robot mode được bật/tắt từ board master 
*/
  if (robot_mode == false) {output_value_index[FC_VALUE_W1]  = bitClear(output_value_index[FC_VALUE_W1],ROBOT_MODE);}  // disable robot mode
  if (robot_mode == true)  {output_value_index[FC_VALUE_W1]  = bitSet(output_value_index[FC_VALUE_W1],ROBOT_MODE);}    // enable robot mode
/* Kiểm tra bit command chạy tới vị trí gắp/thả sản phẩm */
  if(bitRead(output_value_index[FC_VALUE_W2],ARM_SET_REVERS)) {
    output_value_index[FC_VALUE_W1] = bitSet(output_value_index[FC_VALUE_W1],ARM_REVERS);
    output_value_index[FC_VALUE_W1] = bitClear(output_value_index[FC_VALUE_W1],ARM_FORWARD);
  }
  if(bitRead(output_value_index[FC_VALUE_W2],ARM_SET_FORWARD)) {
    output_value_index[FC_VALUE_W1] = bitSet(output_value_index[FC_VALUE_W1],ARM_FORWARD);
    output_value_index[FC_VALUE_W1] = bitClear(output_value_index[FC_VALUE_W1],ARM_REVERS);
  }
/* Đọc giá trị cảm biến kiểm tra vị trí gap/tha của tay máy */
  uint8_t *input_sensor = Read_Input_From_MCP23017();
// Nếu tay máy đang ở vị trí gắp thì không thể chạy về vị trí gắp được nữa
  if (!bitRead(input_sensor[1],SENSOR_POSITION_PICK)) {
    output_value_index[FC_VALUE_W1]  = bitClear(output_value_index[FC_VALUE_W1],ARM_REVERS);
  }
// Nếu tay máy đang ở vị trí thả thì không thể chạy về vị trí thả được nữa
  if (!bitRead(input_sensor[1],SENSOR_POSITION_UNPICK)) {
    output_value_index[FC_VALUE_W1]  = bitClear(output_value_index[FC_VALUE_W1],ARM_FORWARD);
  }
/* Kiểm tra bit chạy tới vị trí gắp/thả trong output_value 
    Tùy cơ cấu tay máy dùng động cơ AC, servo, step để xuất thêm tín hiệu phù hợp  
    Trường hợp dùng AC motor: xả thắng, bật mode tốc độ, bật mode đếm xung encoder
*/
// Nếu tay máy không ở vị trí gắp/thả và có lệnh về vị trí gắp/thả
  if (bitRead(output_value_index[FC_VALUE_W1],ARM_FORWARD) || bitRead(output_value_index[FC_VALUE_W1],ARM_REVERS)) {
      output_value_index[FC_VALUE_W1] = bitSet(output_value_index[FC_VALUE_W1],ARM_UNBREAK);    // Xả thắng
      output_value_index[FC_VALUE_W1] = bitSet(output_value_index[FC_VALUE_W1],ARM_HIGH_SPEED); // Chạy tốc độ cao
      encoder_start_count(LS7366_Motor_AC); // bắt đầu đếm xung trục X (động cơ AC 3pha)
      Init_timer_counter_01();              // Ngắt tràn Timer 1 được sử dụng để đọc tín hiệu encoder
      AC_3phase_motor_run = true;           // set flag cho phép đếm xung trong ngắt timer 1
// Trường hợp không có lệnh về vị trí gắp/thả
  } else {
      output_value_index[FC_VALUE_W1] = bitClear(output_value_index[FC_VALUE_W1],ARM_UNBREAK);    // Tắt xả thắng
      output_value_index[FC_VALUE_W1] = bitClear(output_value_index[FC_VALUE_W1],ARM_HIGH_SPEED); // Tắt chạy tốc độ cao
      output_value_index[FC_VALUE_W1] = bitClear(output_value_index[FC_VALUE_W1],ARM_MID_SPEED);  // Tắt chạy tốc độ trung bình
  }
/* Kiểm tra bit ép sản phẩm trong output_value_index[1] */
  if (bitRead(output_value_index[FC_VALUE_W1], MOLD_CLOSE)) {
      digitalWrite(DONG_KHUON,HIGH); // kích đóng khuôn
      MODE_DONG_KHUON = true;
  }
/* Kiểm tra bit khai báo lấy sản phẩm trong output_value_index[1]
   khi bit này được set 1, sẽ chuyển qua kiểm tra trạng thái mở khuôn hết hay chưa (MO_KHUON_HET set 1 hay chưa)
   nếu MO_KHUON_HET = true, lúc này mới cho phép thực hiện xuất các ouput_coil ra cổng output
*/ 
  if (bitRead(output_value_index[FC_VALUE_W1], GET_PRODUCT)) {
      //MODE_GET_PRODUCT = true;   
      while (!MO_KHUON_HET);       // Chờ đến khi mở khuôn hết
      MO_KHUON_HET = false;        // 
  } 
  output_value[0] = highByte(output_value_index[FC_VALUE_W1]);
  output_value[1] = lowByte(output_value_index[FC_VALUE_W1]);
  output_value[2] = highByte(output_value_index[FC_VALUE_W2]);
  output_value[3] = lowByte(output_value_index[FC_VALUE_W2]);

/* Tạo mã input tương ứng với các output (input mong muốn)*/
  uint8_t *intput_value_index = generate_input_value_sensor(output_value_index);
/* Xuất output_value ra ngõ ouput*/
  Output_port.write_multiple_byte(output_value, wQty);
/*Lưu giá trị cảm biến cần có để dừng tín hiệu output dành cho chế độ auto run*/
for (int i = 0; i < wQty; i++) { value[i] = intput_value_index[i]; }
bool mode_trans_speed = true;
bool sensor_match = true;

while(true){
    sensor_match = true;
    for (int i = 0; i < wQty; i++){
      if (input_sensor[i] != value[i]){           // Chờ đến khi giá trị input_sensor[i] = value[i] 
        sensor_match = false;
      }
    }
    if (AC_3phase_motor_run) {
// Chuyển sang chạy tốc độ trung bình nếu đủ xung.
      if (Ac_motor_trans_speed && mode_trans_speed) { 
        output_value_index[FC_VALUE_W1] = bitSet(output_value_index[FC_VALUE_W1],ARM_MID_SPEED);
        output_value_index[FC_VALUE_W1] = bitClear(output_value_index[FC_VALUE_W1],ARM_HIGH_SPEED);
        output_value[0] = highByte(output_value_index[FC_VALUE_W1]);
        output_value[1] = lowByte(output_value_index[FC_VALUE_W1]);
        Output_port.write_multiple_byte(output_value, wQty);
        mode_trans_speed = false;
      }
// Nếu tay máy đã tới vị trí gắp/thả thì không thể tiếp tục chạy về vị trí gắp/thả được nữa
      if (!bitRead(input_sensor[1],SENSOR_POSITION_PICK) || !bitRead(input_sensor[1],SENSOR_POSITION_UNPICK))  {
        
        output_value_index[FC_VALUE_W1] = bitClear(output_value_index[FC_VALUE_W1],ARM_UNBREAK);    // Tắt xả thắng
        output_value_index[FC_VALUE_W1] = bitClear(output_value_index[FC_VALUE_W1],ARM_HIGH_SPEED); // Tắt chạy tốc độ cao
        output_value_index[FC_VALUE_W1] = bitClear(output_value_index[FC_VALUE_W1],ARM_MID_SPEED);  // Tắt chạy tốc độ trung bình

        if (!bitRead(input_sensor[1],SENSOR_POSITION_PICK)) {
            output_value_index[FC_VALUE_W1] = bitClear(output_value_index[FC_VALUE_W1],ARM_REVERS);}
            
        if (!bitRead(input_sensor[1],SENSOR_POSITION_UNPICK)) { 
            output_value_index[FC_VALUE_W1] = bitClear(output_value_index[FC_VALUE_W1],ARM_FORWARD);}
// Nếu tay máy đi quá hành trình thì reset output coil, ngoại trừ robot mode
        if (!bitRead(input_sensor[1], SENSOR_OVERTRAVEL_PICK) || !bitRead(input_sensor[1], SENSOR_OVERTRAVEL_UNPICK) || exceed_pulse) {  
          Reset_output_coil(); 
          return; }

        output_value[0] = highByte(output_value_index[FC_VALUE_W1]);
        output_value[1] = lowByte(output_value_index[FC_VALUE_W1]);
        Output_port.write_multiple_byte(output_value, wQty);
        encoder_stop_count(LS7366_Motor_AC); // Dừng đếm xung, reset giá trị xung về 0
        Stop_timer_counter_01();             // Dừng ngắt timer 1
        AC_3phase_motor_run = false;
      }
    }
    if (sensor_match) { break;}
// Thoát khỏi while true nếu sensor_match == true: giá trị cảm biến so sánh bằng nhau
    input_sensor = Read_Input_From_MCP23017(); // tiếp tục đọc giá trị input_data nếu sensor_match = false
    delay(1);
  }
} // end void
//================================================================
// Hàm tạo mã input sensor tương ứng với các output coil
// output_value: giá trị coil cần output ra 
uint8_t *generate_input_value_sensor(uint16_t *output_value){
  static uint8_t input_sensor[INPUT_OUTPUT_SIZE-2];
  for (int i = 0; i < 4; i++){input_sensor[i] = 0xFF;}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Kiểm tra bit tay hút: 1: đi xuống; 0: đi lên
  if (bitRead(output_value[FC_VALUE_W2], ARM_SUB_UP_DOWN))
  { bitClear(input_sensor[0],SENSOR_ARM_SUB_DOWN);} else bitClear(input_sensor[0],SENSOR_ARM_SUB_UP);
// Kiểm tra bit tay kẹp: 1: đi xuống; 0: đi lên
//  if (bitRead(output_value[FC_VALUE_W2], ARM_PICK_UP_DOWN)) {bitClear(input_sensor[2],SENSOR_ARM_PICK_DOWN);} else bitClear(input_sensor[2],SENSOR_ARM_PICK_UP);
// Kiểm tra bit tay hút: 1: đi tới; 0: đi lùi
  if (bitRead(output_value[FC_VALUE_W2], ARM_SUB_LEFT_RIGHT)) 
  { bitClear(input_sensor[0],SENSOR_ARM_SUB_LEFT);} else bitClear(input_sensor[0],SENSOR_ARM_SUB_RIGHT);
// Kiểm tra bit tay kẹp: 1: đi tới; 0: đi lùi
//  if (bitRead(output_value[FC_VALUE_W2], ARM_PICK_LEFT_RIGHT)) {bitClear(input_sensor[2],SENSOR_ARM_PICK_LEFT);} else bitClear(input_sensor[2],SENSOR_ARM_PICK_RIGHT);
// Kiểm tra bit tay kẹp 01: 1: kẹp; 0: thả (của mode tay hút)
  if ((bitRead(output_value[FC_VALUE_W2], ARM_PICK_UNPICK_01)) && pick_sensor) {bitClear(input_sensor[0],SENSOR_PICK_01);}
// Kiểm tra bit tay kẹp 02: 1: kẹp; 0: thả (của mode tay kẹp)
  if ((bitRead(output_value[FC_VALUE_W2], ARM_PICK_UNPICK_02)) && pick_sensor) {bitClear(input_sensor[0],SENSOR_PICK_02);} 
// Kiểm tra bit hút chân không 01: 1: hút; 0: thả (của mode tay hút)
  if ((bitRead(output_value[FC_VALUE_W2], ARM_SET_VACUUM_01)) && vaccum_sensor) {bitClear(input_sensor[0],SENSOR_VACCUM_01);}   
// Kiểm tra bit hút chân không 01: 1: hút; 0: thả (của mode tay hút)
  if ((bitRead(output_value[FC_VALUE_W2], ARM_SET_VACUUM_02)) && vaccum_sensor) {bitClear(input_sensor[0],SENSOR_VACCUM_02);}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Kiểm tra bit tay xoay lên
  if (bitRead(output_value[FC_VALUE_W2], ARM_SET_ROTATE_UP)) {bitClear(input_sensor[1],SENSOR_ROTATE_UP);}
// Kiểm tra bit tay xoay xuống
  if (bitRead(output_value[FC_VALUE_W2], ARM_SET_ROTATE_DOWN)) {bitClear(input_sensor[1],SENSOR_ROTATE_DOWN);}
// Kiểm tra bit tới vị trí gắp sản phẩm
  if (bitRead(output_value[FC_VALUE_W2], ARM_SET_REVERS)) {bitClear(input_sensor[1],SENSOR_POSITION_PICK);}
// Kiểm tra bit tới vị trí thả sản phẩm
  if (bitRead(output_value[FC_VALUE_W2], ARM_SET_FORWARD)) {bitClear(input_sensor[1],SENSOR_POSITION_UNPICK);}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Kiểm tra bit đóng khuôn
//  if (bitRead(output_value[FC_VALUE_W1], MOLD_CLOSE)) {bitClear(input_sensor[2],SENSOR_CLOSED_MOLD);} 
// Kiểm tra bit chế độ robot
  if (bitRead(output_value[FC_VALUE_W1], ROBOT_MODE)) {bitClear(input_sensor[2],SENSOR_ROBOT_MODE);}

  return input_sensor;
}
//================================================================
// Thực hiện lệnh INPUT_STEP: ứng với giá trị value -> kiểm tra bit cảm biến tương ứng
// value: giá trị input mong muốn
// wQty: số lượng byte cần đọc từ ngõ input
void Execute_InputStep(uint8_t *value, uint8_t wQty){
    uint8_t *input_data; 
    bool sensor_match; // Biến so sánh giá trị sensor đọc được và value sensor được tạo ra
    input_data = Read_Input_From_MCP23017();

    while(true){
      sensor_match = true;
      for (int i = 0; i < wQty; i++){
        if (input_data[i] != value[i]){           // Chờ đến khi giá trị input_data[i] = value[i] 
          sensor_match = false;
        }
      }
      if (sensor_match) { break; } // Thoát khỏi while true nếu sensor_match == true: giá trị cảm biến so sánh bằng nhau
      input_data = Read_Input_From_MCP23017(); // tiếp tục đọc giá trị input_data nếu sensor_match = false
      delay(20);
    }
}
//================================================================
void function_modbus_slave(void){
    //node_slave.cbVector[CB_READ_COILS] = readDigital;
    //node_slave.cbVector[CB_READ_DISCRETE_INPUTS] = readDigital;
    node_slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;
    //node_slave.cbVector[CB_READ_INPUT_REGISTERS] = readAnalogIn;
    node_slave.cbVector[CB_READ_HOLDING_REGISTERS] = readMemory;
    node_slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = writeMemory;
}
//================================================================
// Configure thanh ghi timer/counter 
// http://arduino.vn/bai-viet/411-timercounter-tren-avrarduino
//================================================================
/* 
TCNTn: Thanh ghi thực hiện đếm giá trị 
TCCRnA/B: Thanh ghi điều khiển:
+ CSn0,CSn1,CSn2: Cài đặt prescale
TIMSKn: Thanh ghi điều khiển ngắt. 
+ bit 5 - ICIEn: Input Capture Interrupt Enable - Cho phép ngắt khi dùng Input Capture.
+ bit 2 - OCIEnB: Output Compare Interrupt Enable 1 channel B -  Cho phép ngắt khi dùng Output Compare ở channel B.
+ bit 1 - OCIEnA: Output Compare Interrupt Enable 1 channel A -  Cho phép ngắt khi dùng Output Compare ở channel A.
+ bit 0 - TOIEn: Overflow Interrupt Enable 1 - Cho phép ngắt khi xảy ra tràn trên T/C.
*/
void Init_timer_counter_02(){ // Normal mode
// Cần tạo ra ngắt tràn trên timer counter sau mỗi 1.6 ms
  noInterrupts();
  TCCR2B = TCCR2A = 0;
  TIMSK2 = 0;
  // prescaler = 256, P_clock=16mhz/256 = 62.5khz: Tần số hoạt động của timer counter 2 là 62.5khz
  // Mỗi xung TCNT2 đếm tăng 1 đơn vị tốn 16 us; TCNT2 = 255 tốn 4080 us = 4.080 ms;
  TCCR2B |=   (1 << CS22) | (1 << CS21); 
  TCNT2 = 155; // timer counter 2 bắt đầu đếm từ giá trị 155, đếm tới 255 được 100 xung: tốn hết 1600 us sẽ tràn
  TIMSK2 |=  (1 << TOIE2); // Cho phép ngắt khi xảy ra tràn trên T/C.
  TIFR2  |= (0 << TOV2);   // TOV2 = 1 sẽ chạy hàm ngắt
  interrupts();
}
//================================================================
// Cứ 1.6 ms hàm ngắt thực hiện 1 lần
ISR(TIMER2_OVF_vect) // Ngat tràn timer 2  TOV2 = 1
{
  TCNT2 = 155;
  node_slave.poll();  // Đưa poll vào hàm ngắt timer 2

  static uint16_t close_mold_counter = 0;
  static bool open_mold_state = false;
  static bool open_mold_mode = false;
                                      
  if (MODE_DONG_KHUON) {                                  // nếu bit đóng khuôn trong output_coil set 1
    digitalWrite(DONG_KHUON,HIGH);                        // kích đóng khuôn
    uint8_t close_mold = digitalRead(CB_DONG_KHUON_HET);  // Đọc cảm biến đóng khuôn hết
    if (close_mold == 0) {close_mold_counter ++;}         // Có tín hiệu đóng khuôn hết, thực hiện đếm tạo thời gian đóng khuôn
    if (close_mold_counter >= CLOSE_MOLD_COUNTER)         // sau thời gian 3750*1.6ms (6s) thì reset đóng khuôn
    { digitalWrite(DONG_KHUON,LOW); close_mold_counter = 0;
      MODE_DONG_KHUON = false; open_mold_mode = true;}
  }
  if (open_mold_mode) {
    uint8_t open_mold = digitalRead(CB_MO_KHUON_HET);     // Đọc cảm biến mở khuôn hết
    if (open_mold == 0) { open_mold_state = true; open_mold_mode = false;} // Có tín hiệu mở khuôn hết
  }
  if (open_mold_state) {
    uint8_t open_mold_done = digitalRead(CB_MO_KHUON_HET);     // Đọc cảm biến mở khuôn hết 
    if (open_mold_done != 0) {open_mold_state = false; MO_KHUON_HET = true;} // Khuôn mở đi qua cb mở khuôn hết
  }

  if (command_modbus == STOP_MODBUS_COMMAND){ 
      Serial.println("STOP");
      Reset_output_coil();
  }
}
//================================================================
//================================================================
void Init_timer_counter_01(){ // Normal mode
// Cần tạo ra ngắt tràn trên timer counter sau mỗi 400 ms
  noInterrupts();
  TCCR1B = TCCR1A = 0;
  TIMSK1 = 0;
  // prescaler = 256, P_clock=16mhz/256 = 62.5khz: Tần số hoạt động của timer counter 1 là hz
  // Mỗi xung TCNT1 đếm tăng 1 đơn vị tốn 16 us; TCNT1 = 65535 tốn 262140 us = 262.140 ms;
  TCCR1B |=   (1 << CS12); 
  TCNT1 = 40536; // timer counter 1 bắt đầu đếm từ giá trị 40536, đếm tới 65535 được 25000 xung: tốn hết 400 ms sẽ tràn
  TIMSK1 |=  (1 << TOIE1); // Cho phép ngắt khi xảy ra tràn trên T/C.
  TIFR1  |= (0 << TOV1);   // TOV1 = 1 sẽ chạy hàm ngắt
  interrupts();
}
void Stop_timer_counter_01(){
  noInterrupts();
  TCCR1B = TCCR1A = 0;
  TIMSK1 = 0;
  interrupts();
}
//================================================================
// Cứ 400 ms hàm ngắt thực hiện 1 lần, dùng trong COMMAND: MONITOR_MODE
//================================================================
// Cứ 1 ms hàm ngắt timer1 thực hiện 1 lần
ISR(TIMER1_OVF_vect) // Ngat tràn timer 2  TOV2 = 1
{
  TCNT1 = 40536;
  //uint8_t *input_data = Read_Input_From_MCP23017();
  if (AC_3phase_motor_run){
  // cứ 1.6 ms đọc giá trị encoder về 
  uint32_t encoder_counter_AC = LS7366_Motor_AC.read_counter_register();
  uint8_t  encoder_status_AC  = LS7366_Motor_AC.read_status_register();

    if (encoder_counter_AC >= pulse_set_AC - 200){
      Ac_motor_trans_speed = true;
    }
    if (encoder_counter_AC > pulse_set_AC){
      exceed_pulse = true;
    }
  }
}
//============================================================================================
//============================================================================================
void setup() {   
  Serial.begin(9600);
  MODBUS_SERIAL.begin(MODBUS_BAUDRATE);
  node_slave.begin(MODBUS_BAUDRATE);
  function_modbus_slave();

  pinMode(DONG_KHUON, OUTPUT);    digitalWrite(DONG_KHUON,LOW);
  pinMode(CHE_DO_ROBOT, OUTPUT);  digitalWrite(CHE_DO_ROBOT,LOW);
  pinMode(COI_BAO, OUTPUT);       digitalWrite(COI_BAO, LOW);
  pinMode(RESET_PIN, OUTPUT);     digitalWrite(RESET_PIN, LOW);

  pinMode(CB_DONG_KHUON_HET, INPUT_PULLUP); 
  pinMode(CB_MO_KHUON_HET, INPUT_PULLUP);
   
  EEPROM256_01.begin(SELECT_EEPROM_25LC256_01);
  EEPROM256_02.begin(SELECT_EEPROM_25LC256_02);

  MCP23017_01.begin(MCP23017_I2C_ADDRESS_01, INPUT_MODE); 
  MCP23017_02.begin(MCP23017_I2C_ADDRESS_02, INPUT_MODE);
/*
  LS7366_Motor_X.begin(SELECT_LS7366_X);
  LS7366_Motor_Y.begin(SELECT_LS7366_Y);
  LS7366_Motor_Z.begin(SELECT_LS7366_Z);
  LS7366_Motor_A.begin(SELECT_LS7366_A);
*/

/* Chỉ dùng cho chế độ chạy stepper và servo motor
  noInterrupts();
  Motor_X.begin(PWM_MOTOR_01, DIR_MOTOR_01); 
  Motor_Y.begin(PWM_MOTOR_02, DIR_MOTOR_02);
  Motor_Z.begin(PWM_MOTOR_03, DIR_MOTOR_03);
  Motor_A.begin(PWM_MOTOR_04, DIR_MOTOR_04);
  interrupts();
*/
  EEPROM24C256_01.begin(ADDR_EEPROM_24C256_01);
  EEPROM24C256_02.begin(ADDR_EEPROM_24C256_02);
  LS7366_Motor_AC.begin(SELECT_LS7366_AC);  // Đọc encoder của động cơ AC trong trường hợp ko dùng servo/stepper
  Output_port.begin(SR_DATA_PIN, SR_CLK_PIN, OE_PIN, LATCH_PIN, RST_PIN);
  Init_timer_counter_02(); // Config ngắt timer 2 để chạy hàm node_slave.poll();
}
//============================================================================================
/* Flow auto run
1. Tay máy về vị trí Home, tay máy ở vị trí waiting_possiton, có tín hiệu robot mode, chờ tín hiệu Start
2. Khi có tín hiệu Start, thực hiện delay step
3. Sau khi delay step hoàn tất, thực hiện output step đầu tiên có tín hiệu kích đóng khuôn
4. Khuôn đóng lại, chờ tín hiệu cảm biến đóng khuôn hết
5. Tiếp tục chờ đến khi mở khuôn, cảm biến mở khuôn hết on -> off (Khuôn mở ra chạy vụt qua cảm biến)
6. Thực hiện delay và đưa tay máy xuống lấy sản phẩm
7. Thực hiện output step và delay step để thực thi các hoạt động của tay máy như đã teach trước đó
8. Khi output step nào đó có tín hiệu đóng khuôn lần 2 thì thực hiện đóng khuôn đồng thời tay máy thực hiện các thao tác khác
9. Tiếp tục chờ khi có tín hiệu mở khuôn hết đồng thời chờ tay máy về lại vị trí chờ lấy sản phẩm
10. 
*/
//============================================================================================
void loop() { 
  //node_slave.poll(); // Đưa poll vào hàm ngắt timer 2
//============================================================================================ 
  if (state_coil == AUTO_MODE_MODBUS_COMMAND && home_position) { 
    digitalWrite(COI_BAO, HIGH); delay(500); digitalWrite(COI_BAO, LOW);
// Đọc giá trị bắt đầu từ địa chỉ program_start_addr trong EEPROM256 và thực thi. 
    uint16_t index_address; state_coil = 0; 
    uint16_t u16_function_value; uint32_t u32_function_value[2]; uint8_t u8_function_value[4];
    bool pick_state = true; bool waiting_close_mold = false;  bool end_program_cycle = false;       
    bool motor_state = false;      
    uint16_t get_function; uint8_t packet_len = 0;    // Số byte cần read từ EEPROM256
    uint8_t  *respond_data; 
    uint8_t respond_data_index[MAX_FARME_DATASIZE];
    uint16_t _output_coil[INPUT_OUTPUT_SIZE/2]; 

    int32_t x_axis_value = 0;
    int32_t y_axis_value = 0;
    int32_t z_axis_value = 0;
    int32_t a_axis_value = 0;
 
    /* Kiểm tra tay máy đã về vị trí HOME (vị trí gắp sản phẩm) hay chưa */
    /* Chế độ robot mode đã được set trong output coil ở vị trí waiting_possiton  */
    index_address = program_start_addr + 4; // Đọc giá trị mode arm
    auto_running = true;                    // Cờ báo đang ở chế độ chạy auto
    while (home_position){ 
/*Tín hiệu MO_KHUON_HET trong ngắt timer được set true, 
tức là đã thực hiện đóng khuôn hết -> ép sản phẩm -> mở khuôn hết
*/   
      get_function = Read_2byte_data_from_EEPROM (eeprom_type, index_address);
      Serial.println("=======================================================================");
      if (!pick_state) { break; } // exit while home position

      while (pick_state) { // Tay máy vào vị trí chờ lấy sản phẩm
/*Kiểm tra chế độ dùng tay máy, tùy vào từng chế độ để kiểm tra input_sensor tương ứng*/
          if (get_function == MODE_ARM) { //FFFF FFFF
            packet_len = MODE_ARM_SIZE;
            respond_data = Read_Program_Function_From_EEPROM (eeprom_type, packet_len ,index_address);

            switch (UInt16_word(respond_data,2)){ 
              case MAIN_ARM_MODE: main_arm_mode = true ; Serial.println("1 - Chế độ tay hút"); break; // Chế độ chỉ kiểm tra input_sensor của tay hút
              case PICK_ARM_MODE: pick_arm_mode = true;  Serial.println("1 - Chế độ tay gắp"); break; // Chế độ chỉ kiểm tra input_sensor của tay kẹp
              case TWO_ARM_MODE:  two_arm_mode = true;   Serial.println("1 - Chế độ kết hợp"); break; // Chế độ kiểm tra input_sensor của cả 2
            }
            index_address += packet_len;
            // Lưu respond_data vào respond_data_index để truyền tới board master
            for (int i = 0; i < packet_len; i++) { respond_data_index[i] = respond_data[i]; }
            Save_packet_data_to_feedback_register(packet_len, respond_data_index);

            delay(500);
            break;  
          }
/*Kiểm tra có dùng cảm biến chân không hay không*/
          if (get_function == VACCUM_SENSOR) { //FFFF FFFF
            packet_len = VACCUM_SENSOR_SIZE;
            respond_data = Read_Program_Function_From_EEPROM (eeprom_type, packet_len, index_address);
            switch (UInt16_word(respond_data,2)){
                case 0:  vaccum_sensor = true; Serial.println("2 - Use sensor"); break;
                case 1:  vaccum_sensor = false; Serial.println("2 - No sensor");  break;
            }
            index_address += packet_len; 
            // Lưu respond_data vào respond_data_index để truyền tới board master
            for (int i = 0; i < packet_len; i++) { respond_data_index[i] = respond_data[i]; }
            Save_packet_data_to_feedback_register(packet_len, respond_data_index);

            delay(500);
            break;
          } 
/*Kiểm tra có dùng cảm biến tay kẹp hay không*/
          if (get_function == PICK_SENSOR) { // FFFF FFFF
            packet_len = VACCUM_SENSOR_SIZE;
            respond_data = Read_Program_Function_From_EEPROM (eeprom_type, packet_len, index_address);
            switch (UInt16_word(respond_data,2)){
                case 0:  pick_sensor = true;  Serial.println("2 - Use sensor"); break;
                case 1:  pick_sensor = false; Serial.println("2 - No sensor");  break;
            }
            index_address += packet_len; 
            // Lưu respond_data vào respond_data_index để truyền tới board master
            for (int i = 0; i < packet_len; i++) { respond_data_index[i] = respond_data[i]; }
            Save_packet_data_to_feedback_register(packet_len, respond_data_index);

            delay(500);
            break;
          }
/*Ở waiting_possiton, robot mode đã được on*/
          if (get_function == WAITING_POSSITON) { //FFFF FFFF FFFF
            packet_len = INPUT_OUTPUT_SIZE;
            respond_data = Read_Program_Function_From_EEPROM (eeprom_type, packet_len, index_address);
            for (int i = 0; i < packet_len - 2; i++){
              u8_function_value[i] = respond_data[i+2];  // lấy giá trị 
            }
            // Lưu respond_data vào respond_data_index để truyền tới board master
            for (int i = 0; i < packet_len; i++) { respond_data_index[i] = respond_data[i]; }
            Save_packet_data_to_feedback_register(packet_len, respond_data_index);
            // Thực hiện output_coil
            Execute_OutputStep(u8_function_value, packet_len - 2); 
            Serial.println("3 - Vào vị trí waiting_possiton");
            index_address += packet_len;
            delay(500);
            break;
          }
/*Kiểm tra delay*/
          if (get_function == DELAY_STEP) { //FFFF FFFF
            packet_len = STEP_REPEAT_DELAY_SIZE;
            respond_data = Read_Program_Function_From_EEPROM (eeprom_type, packet_len ,index_address);
            u16_function_value = UInt16_word(respond_data, 2); // lấy giá trị delay
            // Lưu respond_data vào respond_data_index để truyền tới board master
            for (int i = 0; i < packet_len; i++) { respond_data_index[i] = respond_data[i]; }
            Save_packet_data_to_feedback_register(packet_len, respond_data_index);
            // Thực hiện delay
            Execute_DelayStep(u16_function_value); 
            Serial.println("4 - Delay");          
            index_address += packet_len;
            delay(500);
            break;
          } 
/*Thực hiện output*/          
          if (get_function == OUTPUT_STEP) { //FFFF FFFF FFFF
            packet_len = INPUT_OUTPUT_SIZE;
            respond_data = Read_Program_Function_From_EEPROM (eeprom_type, packet_len ,index_address);
            for (int i = 0; i < packet_len - 2; i++){
              u8_function_value[i] = respond_data[i+2];
            }
            // Lưu respond_data vào respond_data_index để truyền tới board master
            for (int i = 0; i < packet_len; i++) { respond_data_index[i] = respond_data[i]; }
            Save_packet_data_to_feedback_register(packet_len, respond_data_index);
            // Thực hiện output coil
            Execute_OutputStep(u8_function_value, packet_len - 2);
            index_address += packet_len;
            delay(500);
            break;
          }
/*Kiểm tra mã dừng chương trình, quay về vị trí chờ lấy sản phẩm*/
          if (get_function == REPEAT_STEP) { //FFFF
            packet_len = STEP_REPEAT_DELAY_SIZE;  // 4 byte: 2 byte đầu lưu mã nhận diện, 2 byte sau lưu giá trị
            respond_data = Read_Program_Function_From_EEPROM (eeprom_type, packet_len ,index_address);
            // Lưu respond_data vào respond_data_index để truyền tới board master
            for (int i = 0; i < packet_len; i++) { respond_data_index[i] = respond_data[i]; }
            Save_packet_data_to_feedback_register(packet_len, respond_data_index);

            pick_state = false;
            index_address = program_start_addr + 4;  // Quay về địa chỉ index_address đầu chương trình
            delay(500);  
            break;
          }
      } // end pick_state

      while(motor_state) {

        if (get_function == PULSE_MOTOR){ // FFFF FFFF FFFF FFFF FFFF FFFF FFFF FFFF FFFF 18 bytes
            packet_len = PULSE_MOTOR_SIZE;
            respond_data = Read_Program_Function_From_EEPROM (eeprom_type, packet_len ,index_address);

            u32_function_value[0] = UInt16_word(respond_data, 2);
            u32_function_value[1] = UInt16_word(respond_data, 4);
            x_axis_value = ((u32_function_value[0] << 16) | u32_function_value[1]);

            u32_function_value[0] = UInt16_word(respond_data, 6);
            u32_function_value[1] = UInt16_word(respond_data, 8);
            y_axis_value = ((u32_function_value[0] << 16) | u32_function_value[1]);

            u32_function_value[0] = UInt16_word(respond_data, 10);
            u32_function_value[1] = UInt16_word(respond_data, 12);
            z_axis_value = ((u32_function_value[0] << 16) | u32_function_value[1]);    

            u32_function_value[0] = UInt16_word(respond_data, 14);
            u32_function_value[1] = UInt16_word(respond_data, 16);
            a_axis_value = ((u32_function_value[0] << 16) | u32_function_value[1]);

            Execute_XaxisStep(x_axis_value, 1000); Execute_YaxisStep(y_axis_value, 1000); 
            Execute_ZaxisStep(z_axis_value, 1000); Execute_AaxisStep(a_axis_value, 1000);
            index_address = index_address + packet_len;   
        }
      } // end motor_state
    } // end home_position
  } // End if AUTO_MODE_MODBUS_COMMAND
//================================================================================================
// Trước khi chạy auto run, tay máy phải về vị trí HOME.
// Đưa tay máy về vị trí HOME (vị trí gắp sản phẩm)
//================================================================================================
  if (command_modbus == HOME_MODBUS_COMMAND && !auto_running){
    command_modbus = 0xFFFF;
    digitalWrite(COI_BAO, HIGH); delay(500); digitalWrite(COI_BAO, LOW);
    uint16_t index_address;  uint16_t get_function;
    uint8_t packet_len = 4; uint8_t *respond_data; uint8_t respond_data_index[MAX_FARME_DATASIZE];
    uint8_t u8_function_value[4];
    bool home_state = true; 
    bool setup_state = true;
    Serial.println("Go to HOME ...");
    /* Reset các output valve 
    Đưa tay máy về vị trí home (vị trí gắp)
    */
    for (int i = 0; i < 4; i++) {u8_function_value[i] = 0;}
    u8_function_value[2] = bitSet(u8_function_value[2],ARM_SET_REVERS - 8); // về vị trí gắp sản phẩm
    robot_mode = true;
    Execute_OutputStep(u8_function_value, packet_len);
    digitalWrite(COI_BAO, HIGH); delay(500); digitalWrite(COI_BAO, LOW);
    address_run_auto = index_address;
    state_coil = 0;
    home_position = true; // đã ở vị trí home
  } // end if HOME
//============================================================================================
// Kiểm tra mã command_modbus để thực hiện lệnh ghi vào eeprom
  Save_receive_buffer_data_to_eeprom();
} // End loop
//============================================================================================
//============================================================================================
// Reset tay máy, vẫn để bật chế độ robot mode
//============================================================================================
void Reset_output_coil(){
    uint8_t u8_function_value[4];
    uint8_t packet_len = 4;
    digitalWrite(COI_BAO, HIGH); delay(100); digitalWrite(COI_BAO, LOW);
    for (int i = 0; i < 4; i++) {u8_function_value[i] = 0;}
    u8_function_value[1] = bitSet(u8_function_value[1], ROBOT_MODE);
    Output_port.write_multiple_byte(u8_function_value, packet_len);
    digitalWrite(COI_BAO, HIGH); delay(100); digitalWrite(COI_BAO, LOW);
    digitalWrite(RESET_PIN, HIGH); // reset hardware slave board
}
//============================================================================================
// Lưu mã function vào feedback_data
// Dùng để gửi giá trị về master trong chế độ run auto, monitor
//============================================================================================
void Save_packet_data_to_feedback_register(uint8_t packet_len, uint8_t *packet_data){
    for (int i = 0; i < packet_len; i++ ){
      feedback_data[i] = packet_data[i];
    }
    ready_send = true; // Sẵn sàng để cho read_data từ board master sau khi đã hoàn tất lưu packet_data vào feedback_data
}
//============================================================================================
// Lưu data nhận được từ master vào eeprom
//============================================================================================
void Save_receive_buffer_data_to_eeprom(){

  if (command_modbus == WRITE_EEPROM_MODBUS_COMMAND) { // Ghi packet_data vào eeprom
    command_modbus = 0xFFFF;
    Serial.println("Lưu program function vào eeprom");
    Serial.print("Gia tri data: "); 
    for (int i = 0; i < length_write_to_eeprom; i++)
    {
      Serial.print(u8receive_buffer_data[i], HEX); Serial.print("_");
    } 
    Serial.println("");
    Serial.print("Địa chỉ address: "); Serial.println(address_write_to_eeprom,HEX);
    Serial.print("Số lượng bytes: "); Serial.println(length_write_to_eeprom);
    Serial.println("===========================================");
    Write_Program_Function_to_EEPROM(eeprom_type, length_write_to_eeprom, address_write_to_eeprom, u8receive_buffer_data);
  }

  if (command_modbus == WRITE_EEPROM_SINGLE_REG_MODBUS_COMMAND) { // Ghi 2 byte data vào eeprom
    command_modbus = 0xFFFF;
    Serial.println("Lưu 2bytes data vào eeprom");
    Serial.print("Gia tri data: "); Serial.println(u16receive_buffer_data[SINGLE_REGISTER_ADDR], HEX);
    Serial.print("Địa chỉ address: "); Serial.println(address_write_to_eeprom,HEX);
    Write_2byte_data_to_EEPROM(eeprom_type, u16receive_buffer_data[SINGLE_REGISTER_ADDR], address_write_to_eeprom);
    Serial.println("===========================================");
  }
}
//============================================================================================
// Handle the function codes Write Holding Register(s) (FC=06, FC=16) and write data to the eeprom.
// Ghi giá trị vào eeprom địa chỉ bắt đầu từ address. số lượng data 16 bit: length
//============================================================================================
uint8_t writeMemory(uint8_t fc, uint16_t address, uint16_t length)
{
   uint16_t read_data[length]; 
   uint8_t  save_data[2*length]; 
   uint8_t  count = 0;
   
    // Write the received data to EEPROM.
    for (int i = 0; i < length; i++) {
      // Read the value from the input buffer.
        read_data[i] = node_slave.readRegisterFromBuffer(i);
        save_data[count++] = highByte(read_data[i]);
        save_data[count++] = lowByte(read_data[i]);
        u16receive_buffer_data[i] = read_data[i];
    }

    length_write_to_eeprom = 2*length; // số lượng byte đọc được
    address_write_to_eeprom = address; // địa chỉ 

    for (int j = 0; j <  2*length; j++) { u8receive_buffer_data[j] = save_data[j];}  // Lưu giá trị byte nhận được vào bộ nhớ đệm
// Lựa chọn epprom để đọc/ghi
    if (state_coil == CHOOSE_EEPROM_MODBUS_COMMAND) { //0000 0000 0000 1000: bit thứ 3 của state_coil set 1
        choose_eeprom_type = read_data[SINGLE_REGISTER_ADDR];
        switch (choose_eeprom_type){
              case CHOOSE_EEPROM_01: eeprom_type = EEPROM24C256_01; break;
              case CHOOSE_EEPROM_02: eeprom_type = EEPROM24C256_02; break;  
        }
        Serial.print("Choose_eeprom_type: "); Serial.println(read_data[SINGLE_REGISTER_ADDR],HEX);
        return STATUS_OK;}
// Lưu giá trị địa chỉ đầu tiên để đọc lúc auto run
    if (state_coil == PROGRAM_START_MODBUS_COMMAND) { //0000 0000 0000 0100: bit thứ 2 của state_coil set 1
        program_start_addr = read_data[SINGLE_REGISTER_ADDR]; 
        Serial.print("Địa chỉ program_start_addr: "); Serial.println(program_start_addr,HEX);
        return STATUS_OK;}
// Ghi multiple data vào eeprom
    if (state_coil == WRITE_EEPROM_MODBUS_COMMAND) { //0000 0000 0000 0001: bit thứ 0 của state_coil set 1
        command_modbus = WRITE_EEPROM_MODBUS_COMMAND;
        return STATUS_OK;}
// Ghi 2 byte vào eeprom    
    if (state_coil == WRITE_EEPROM_SINGLE_REG_MODBUS_COMMAND) { //0000 0000 0001 0000: bit thứ 4 của state_coil set 1 // Ghi 2 byte (1 word) vào eeprom
        command_modbus = WRITE_EEPROM_SINGLE_REG_MODBUS_COMMAND;
        return STATUS_OK;}
// Ghi multiple data vào thanh ghi output_coil
    if (state_coil == WRITE_MULTI_OUTPUT_REG_MODBUS_COMMAND){ // bit thứ 5 - nhận giá trị output function từ master
      for (int i = 0; i < 2*length - 2; i++){
          output_coil[i] = save_data[i+2]; // lấy giá trị word thứ 1 và 2 của read_data
      }
        Execute_OutputStep (output_coil, INPUT_OUTPUT_SIZE - 2); 
        return STATUS_OK;
        }
  return STATUS_OK;     
}
//============================================================================================
// Handle the function code Read Holding Registers (FC=03) and write back the values from the EEPROM (holding registers).
//============================================================================================
uint8_t readMemory(uint8_t fc, uint16_t address, uint16_t length)
{
    uint16_t value[length]; uint8_t *data_read;
    // Read the requested EEPROM registers.
    while (true){
// Đọc multiple data từ eeprom
        if (state_coil == READ_EEPROM_MODBUS_COMMAND) { //0000 0000 0000 0010: bit thứ 1 của state_coil set 1
          data_read = Read_Program_Function_From_EEPROM (eeprom_type, 2*length, address);
          for (int i = 0; i < length; i++)
          {  
            value[i] = UInt16_word(data_read,2*i);
            node_slave.writeRegisterToBuffer(i, value[i]); // Write 16 bit data
          }
        break; // thoat khỏi vòng lăp while
        } 
// Đọc n byte data từ thanh ghi feedback_data
        if (state_coil == READ_REGISTER_MODBUS_COMMAND) { 
          Serial.println("write feedback_data to master");
          // gán giá trị vào data_read
          for (int i = 0; i < length; i++){
            value[i]  = UInt16_word(feedback_data,2*i);
            //Serial.print(value[i],HEX);
            node_slave.writeRegisterToBuffer(i, value[i]); // Write 16 bit data
          }
        ready_send = false;   
        break; // thoat khỏi vòng lăp while
        }
      break; // thoat khỏi vòng lăp while
    }
  return STATUS_OK;
}
//============================================================================================
// Handle the function codes Force Single coil (FC=05) and Force Multiple Coils (FC=15) and set the digital output pins (coils).
// Địa chỉ coil là số thứ tự trong mảng coil: 0 - 9
// coil[0]: lựa chọn EEPROM để ghi 
//============================================================================================
uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length)
{
   // Check if the requested addresses exist in the array
    if (address > coil_size || (address + length) > coil_size)
    {
        return STATUS_ILLEGAL_DATA_ADDRESS;
    }
    // Set the output pins to the given state.
    for (int i = 0; i < length; i++)
    {
        // Write the value in the input buffer to the digital pin.
         coil[address] = node_slave.readCoilFromBuffer(i);
    }
    if (coil[address] == 1){state_coil = bitSet(state_coil, address); }
    else state_coil = bitClear(state_coil, address);

    if (state_coil == HOME_MODBUS_COMMAND) {command_modbus = HOME_MODBUS_COMMAND; Serial.println("HOME");}
    // Kích chân reset board
    if (state_coil == STOP_MODBUS_COMMAND) {command_modbus = STOP_MODBUS_COMMAND; Serial.println("STOP");}
    
   // Serial.print("coil address: "); Serial.println(state_coil,HEX);
    
    return STATUS_OK;
}