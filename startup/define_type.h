/*  define_type.h
*/

#ifndef __define_type
#define __define_type

  
#define     __I     volatile const            /*!< defines 'read only' permissions      */
#define     __O     volatile                  /*!< defines 'write only' permissions     */
#define     __IO    volatile                  /*!< defines 'read / write' permissions   */

#define uint32_t unsigned int
#define uint16_t unsigned short
#define uint8_t unsigned char

#define ex_uint32_t extern unsigned int
#define ex_uint16_t extern unsigned short
#define ex_uint8_t extern unsigned char

#define const_uint32_t const unsigned int

#define TRUE    1
#define FALSE   0

#define ui32    uint32_t
#define ui16    uint16_t
#define ui8     uint8_t


#define u32    uint32_t
#define u16    uint16_t
#define u8     uint8_t


#endif /* __define_type */