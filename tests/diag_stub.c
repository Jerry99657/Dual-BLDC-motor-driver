/* Other suites isolate diagnostic recording; diag_host exercises the real
 * module and irq_host verifies the entry/exit instrumentation separately. */
#include "board_diag.h"
#include <string.h>
uint32_t SystemCoreClock=170000000U;
bool diag_host_clear_allowed=true;
unsigned diag_host_clears,diag_host_faults,diag_host_timings;
unsigned diag_host_outer;uint32_t diag_host_at,diag_host_age;
void BoardDiag_Init(void) {}
void BoardDiag_OuterReset(void) {}
bool BoardDiag_Clear(void) { if(diag_host_clear_allowed) ++diag_host_clears; return diag_host_clear_allowed; }
void BoardDiag_Motion(unsigned axis,unsigned mode,float a,float b,float c,float d,float e)
{ (void)axis;(void)mode;(void)a;(void)b;(void)c;(void)d;(void)e; }
void BoardDiag_Fault(unsigned axis,unsigned source,const char *reason)
{ (void)axis;(void)source;(void)reason;++diag_host_faults; }
void BoardDiag_Timing(unsigned outer,uint32_t at,uint32_t age)
{ diag_host_outer=outer;diag_host_at=at;diag_host_age=age;++diag_host_timings; }
const char *BoardDiag_Row(unsigned axis,unsigned row,char value[32])
{ (void)axis;(void)row;strcpy(value,"--");return "Diag"; }
