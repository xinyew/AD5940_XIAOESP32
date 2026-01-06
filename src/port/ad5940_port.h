#ifndef _AD5940_PORT_H_
#define _AD5940_PORT_H_

#include "../ad5940lib/ad5940.h"

#ifdef __cplusplus
extern "C" {
#endif

// AD5940_MCUResourceInit is declared in ad5940.h
/* Removed conflicting declarations.
   These are declared in ad5940.h which now has C linkage.
   The implementation in ad5940_port.cpp will match.
*/
/* Functions to control AD5940 GPIOs */
void AD5940_CsClr(void);
void AD5940_CsSet(void);
void AD5940_RstSet(void);
void AD5940_RstClr(void);
void AD5940_Delay10us(uint32_t time);
uint32_t AD5940_ReadReg(uint16_t RegAddr);
void AD5940_WriteReg(uint16_t RegAddr, uint32_t RegData);
uint32_t AD5940_GetMCUIntFlag(void);
uint32_t AD5940_ClrMCUIntFlag(void);

void AD5940_Print(const char *fmt, ...);
void AD5940_PrintFloat(float val);
void AD5940_PrintInt(uint32_t val);

#ifdef __cplusplus
}
#endif

#endif /* _AD5940_PORT_H_ */
