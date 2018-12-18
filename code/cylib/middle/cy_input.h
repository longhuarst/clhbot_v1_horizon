#ifndef __cy_input_h
#define __cy_input_h












typedef struct {

	int a;
}cy_input_type;









extern cy_input_type cy_input_;











void cy_input_init(void);
void cy_input_polling(void);








#endif //__cy_input_h
