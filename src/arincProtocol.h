#ifndef __ARINC_PROTOCOL_H__
#define __ARINC_PROTOCOL_H__


#define ARINC_CMD_TIMEOUT                       5       // таймаут ожидания конца пакета ARINC
#define ARINC_SEND_STATE_PERIOD                 20      // интервал передачи пакета состояния



// Команда управления
#define LABEL_CMD                               0xC0    // командное слово
#define LABEL_CMD_COURSE_ANGEL                  0xC2    // курсовой угол
#define LABEL_CMD_TANGAGE_ANGEL                 0xC3    // тангажный угол
#define LABEL_CMD_COURSE_VELOCITY               0xC5    // угловая скорость по курсу
#define LABEL_CMD_TANGAGE_VELOCITY              0xC6    // угловая скорость по тангажу 

// Слова состояния
#define LABEL_STATE_GS                          0xC1     // слово состояние
#define LABEL_STATE_ALIGMENT_COURSE_ANGEL       0x8C     // текущее значение угла юстировки по курсу
#define LABEL_STATE_ALIGMENT_TANGAGE_ANGEL      0x8E     // текущее значение угла юстировки по тангажу
#define LABEL_STATE_COURSE_ANGEL                0xC2     // текущее значение угла по курсу
#define LABEL_STATE_TANGAGE_ANGEL               0xC3     // текущее значение угла по тангажу
#define LABEL_STATE_COURSE_VELOCITY             0xC5     // текущее значение угловой скорости по курсу
#define LABEL_STATE_TANGAGE_VELOCITY            0xC6     // текущее значение угловой скорости по тангажу

// Битовые маски для команд
#define ARINC_CMD_ZKN                           1<<12    // запретить контроль
#define ARINC_CMD_RKN                           1<<13    // разрешить контроль
#define ARINC_CMD_KN                            1<<14    // контроль
#define ARINC_CMD_VUS                           1<<15    // внешнее управление по угловым скоростям
#define ARINC_CMD_VOU                           1<<16    // внешнее управление ориентацией ГС
#define ARINC_CMD_AR                            1<<17    // установка нулей
#define ARINC_CMD_UST0                          1<<18    // арретирование
#define ARINC_CMD_TP                            1<<19    // транспортное положение





// Матрица состояния командного слова 
#define UI_CMD_WORD_MATRIX_STATE_NORMAL            0x00
#define UI_CMD_WORD_MATRIX_STATE_NO_DATA           0x01
#define UI_CMD_WORD_MATRIX_STATE_FUNC_TEST         0x02
#define UI_CMD_WORD_MATRIX_STATE_WARNING           0x03
// УИ,Матрица состояния слова с данными (команды)
#define UI_DATA_WORD_MATRIX_STATE_NORMAL           0x03
#define UI_DATA_WORD_MATRIX_STATE_NO_DATA          0x01
#define UI_DATA_WORD_MATRIX_STATE_FUNC_TEST        0x02
#define UI_DATA_WORD_MATRIX_STATE_WARNING          0x00

// Матрица состояния КИ
#define KI_STATE_WORD_MATRIX_STATE_NORMAL          0x00
#define KI_STATE_WORD_MATRIX_STATE_NO_DATA         0x01
#define KI_STATE_WORD_MATRIX_STATE_FUNC_TEST       0x02
#define KI_STATE_WORD_MATRIX_STATE_WARNING         0x03
// КИ, данные
#define KI_DATA_WORD_MATRIX_STATE_NORMAL           0x03
#define KI_DATA_WORD_MATRIX_STATE_NO_DATA          0x01
#define KI_DATA_WORD_MATRIX_STATE_FUNC_TEST        0x02
#define KI_DATA_WORD_MATRIX_STATE_WARNING          0x00





// Тестовые значения в функциональном тесте
#define ARINC_CMD_ANGLE_TEST_VALUE              16      // в конмаде передаются угла x градусов
#define ARINC_CMD_VELOCITY_TEST_VALUE           8       // в команде передаются скорости x град/сек


// Контроль идентификаторов принятых командных слов
#define ARINC_WORD_ID_IS_CORRECT(word)          ((word>>8)&0x03==0x03)
#define ARINC_WORD_GET_MATRIX(word)             ((word>>29)&0x3)
// Что бы определеить все ли слова в пакете имеют одну и ту же матрицу состояния, сжимаем матрицы всех 5 слов в одно число и справниваем его с одной из констант
#define GET_ALL_WORDS_MATRIX_MASK(m1,m2,m3,m4,m5)   (m1<<0|m2<<2|m3<<4|m4<<6|m5<<8) // маска всех слов
#define _ALL_MATRIX(cmdMatrix, dataMatrix)       GET_ALL_WORDS_MATRIX_MASK(cmdMatrix,dataMatrix,dataMatrix,dataMatrix,dataMatrix)
#define ALL_WORDS_MATRIX_IS_WARNING_MASK        _ALL_MATRIX(UI_CMD_WORD_MATRIX_STATE_WARNING,   UI_DATA_WORD_MATRIX_STATE_WARNING)
#define ALL_WORDS_MATRIX_IS_NORMAL_MASK         _ALL_MATRIX(UI_CMD_WORD_MATRIX_STATE_NORMAL,    UI_DATA_WORD_MATRIX_STATE_NORMAL)
#define ALL_WORDS_MATRIX_IS_FUNC_TEST_MASK      _ALL_MATRIX(UI_CMD_WORD_MATRIX_STATE_FUNC_TEST, UI_DATA_WORD_MATRIX_STATE_FUNC_TEST)
#define ALL_WORDS_MATRIX_IS_NO_DATA_MASK        _ALL_MATRIX(UI_CMD_WORD_MATRIX_STATE_NO_DATA,   UI_DATA_WORD_MATRIX_STATE_NO_DATA)

void arinc_processRxWord(uint32_t word);        // обработать принятое слово


void arincService();
void arincGS_onCmdTimeout();                    // таймаут ожидания команды
void arincGS_onCmdReceived();                   // принята новая команда
void arincGS_processCmd();
//void arincGS_processCurrentCmd(uint32_t cmdWord, int32_t newCourseAngle, int32_t newCourseVelocity,
//                               int32_t tangAngle, int32_t tangVelocity128); // обработать принятую команду



void cmd_RKN();
void cmd_ZKN();
void cmd_KN();
void cmd_UST0();
void cmd_VUS(int16_t _courseVelocity128, int16_t _tangageVelocity128);
void cmd_VUO(int16_t _courseAngle128, int16_t _tangAngle128, int16_t _courseVelocity128, int16_t _tangVelocity128);
void cmd_AR(int16_t _courseAngle128, int16_t _tangAngle128);
void cmd_TP();




uint8_t updateChannelState(uint8_t cmdIsFault); // проверка канала приема

void arincGS_sendState();                       // отправить текущее состояние
uint32_t getGSStateWord();
uint32_t getAligmentCourseAngel();
uint32_t getAligmentTangageAngel();
uint32_t getCourseAngel();
uint32_t getTangageAngel();
uint32_t getCourseVelocity();
uint32_t getTangageVelocity();




#endif //__ARINC_PROTOCOL_H__ 