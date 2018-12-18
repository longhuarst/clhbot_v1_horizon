#ifndef __cy_timer_h
#define __cy_timer_h












typedef struct {
	
	int a;

}cy_timer_type;









extern cy_timer_type cy_timer_;











void cy_timer_init(void);
void cy_timer_polling(void);








#endif //__cy_timer_h
