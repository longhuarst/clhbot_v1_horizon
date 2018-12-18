#ifndef __cy_abb_h
#define __cy_abb_h












typedef struct {
	int a;

}cy_abb_type;









extern cy_abb_type cy_abb_;











void cy_abb_init(void);
void cy_abb_polling(void);

extern void abb_move(float x, float y, float z);
extern void abb_move2(float x, float y, float z);
extern void abb_move_fast(float x, float y, float z);




#endif //__cy_abb_h
