#include <csetjmp>
#include <stdint.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>


// For calling to/from CakeML
extern "C" int cml_main(int argc, char **argv);
extern "C" void cml_exit(void);
std::jmp_buf env;

// Expose FFIs to CakeML
extern "C" void fficonst(int32_t *c, long clen, int32_t *a, long alen);
extern "C" void ffisense(int32_t *c, long clen, int32_t *a, long alen);
extern "C" void ffictrl(int32_t *c, long clen, int32_t *a, long alen);
extern "C" void ffiextCtrl(int32_t *c, long clen, int32_t *a, long alen);
extern "C" void ffiactuate(char *c, long clen, int32_t *a, long alen);
extern "C" void ffistop(int32_t *c, long clen, int8_t *a, long alen);
extern "C" void ffiviolation(char *c, long clen, int32_t *a, long alen);

/* CSV format:
OLD:
5 consts:  A B cirTol dirTol xTol yTol
5 sensors: dx dy v xg yg
6 control: a dx dy w xg yg

CURRENT:
2 consts: T eps
4 sonsers: t v xg yg
7 ctrl: a k t vh vl xg yg

Units:
Accelerations  A B:  cm/s^2
Direction vector (dx,dy): Normalized to 100, (e.g. (100,0) is +x-axis).
This is from the perspective of the *waypoint/goal* pointing *toward*
the vehicle. For example (0,-100) if the waypoint is straight ahead.
Velocity v: cm/s.
Position (xg,yg): cm, relative to the position *and* orientation of the vehicle.
So (-1,0) is 1cm left of the vehicle, (0,1) is one cm dead ahead.
Direction tolerance dirTol: The *squared* magnitude of (dx,dy) may differ from 100 by at most dirTol.
cirTol: cm. This is how far the vehicle is allowed to be from the "perfect" arc path.
xTol, yTol: Units unclear. This is "how false" the other invariants for the physics are allowed to be. We will determine experimentally.

*/
/* 2 consts: T eps
4 sonsers: t v xg yg
7 ctrl: a k t vh vl xg yg
*/
void printConsts(int T, int eps) {
  printf("t(T),v(eps),xg,yg,a,k,t,vh,vl,xg,yg\n");
  printf("%d,%d\n", T, eps);
}

void printSensors(int t, int v, int xg, int yg) {
  printf("%d,%d,%d,%d,",t,v,xg,yg);
}

void printCtrl(int a, int k, int t, int vh, int vl, int xg, int yg) {
  printf("%d,%d,%d,%d,%d,%d,%d\n",a,k,t,vh,vl,xg,yg);
}

void fficonst(int32_t *c, long clen, int32_t *a, long alen) {
  assert(clen == 0);
  assert(alen == 2 * 4);

 /*
  * Insert code for computing the constants here
  */

  a[0] = 10;
  a[1] = 25;
  printConsts(a[0],a[1]);
  
  // Set the values of constants
  /*  a[0] = 100; // sets A
  a[1] = 200; // sets B
  a[2] = 50; // sets cirTol
  a[3] = 10; // sets dirTol
  a[4] = 10; // sets xTol
  a[5] = 10; // sets yTol*/
  //  printConsts(a[0],a[1],a[2],a[3],a[4],a[5]);
}

void ffisense(int32_t *c, long clen, int32_t *a, long alen) {
  assert(clen == 0);
  assert(alen == 4 * 4);

 /*
  * Insert code for computing the current sensor values here
  */

  // Set the current sensor values
    a[0] = 0; // sets t
  a[1] = 1; // sets v
  a[2] = 0; // sets xg
  a[3] = 100; // sets yg
  printSensors(a[0],a[1],a[2],a[3]);
  // printf("SENSE: dx=%d, dy=%d, v=%d, xg=%d, yg=%d\n",a[0],a[1],a[2],a[3],a[4]);
}

void ffiextCtrl(int32_t *c, long clen, int32_t *a, long alen) {
  //  assert(clen == 11 * 4);
  assert(alen == 7 * 4);

  // the constants
  /*  int32_t A = c[0]; // the current A
  int32_t B = c[1]; // the current B
  int32_t cirTol = c[2]; // the current cirTol
  int32_t dirTol = c[3]; // the current dirTol
  int32_t xTol = c[4]; // the current xTol
  int32_t yTol = c[5]; // the current yTol
  // the current sensor values
  int32_t dx = c[6]; // the current dx
  int32_t dy = c[7]; // the current dy
  int32_t v = c[8]; // the current v
  int32_t xg = c[9]; // the current xg
  int32_t yg = c[10]; // the current yg*/

 /*
  * Insert code for computing the (unverified) control values here
  */

  // Set the control values
  a[0] = 1; // sets a
  a[1] = 0;   // sets k
  a[2] = 0; // sets t
  a[3] = 0;  // sets vh
  a[4] = 100;  // sets vl
  a[5] = 0; // sets xg
  a[6] = 100; // sets yg
  printCtrl(a[0],a[1],a[2],a[3],a[4],a[5],a[6]);
  //  printf("extCtl: a=%d, dx=%d, dy=%d, w=%d, xg=%d, yg=%d\n",a[0],a[1],a[2],a[3],a[4],a[5]);
}

void ffictrl(int32_t *c, long clen, int32_t *a, long alen) {
  ffiextCtrl(c,clen,a,alen);
}

void ffiactuate(char *c, long clen, int32_t *a, long alen) {
  assert(alen == 7 * 4);

  // the actuation values
  int32_t aa = a[0]; // the current a
  int32_t dx = a[1]; // the current dx
  int32_t dy = a[2]; // the current dy
  int32_t w = a[3]; // the current w
  int32_t xg = a[4]; // the current xg
  int32_t yg = a[5]; // the current yg

    const char* how = (const char *)c; // distinguish between normal OK and fallback
  if (strncmp(how,"OK",clen) == 0) {
    //    printf("OK\n");
    // Control monitor OK
  } else if (strncmp(how,"Control Violation",clen) == 0) {
    //    printf("CV\n");
    // Control monitor violated
  } else {
    printf("HUH\n");
    // Unknown string -- should never occur
    assert(false);
  }

 /*
  * Insert code for actuating the controls
  */
}

void ffistop(int32_t *c, long clen, int8_t *a, long alen) {
    static int x = 100;
  assert(clen == 0);
  assert(alen == 1);

  bool stop = !(x--);

  /*
   * Insert code for deciding whether to continue running here
   */

  if (stop) {
    //      printf("STOP\n");
    a[0] = 1;
  }
  else
    a[0] = 0;
}

void ffiviolation(char *c, long clen, int32_t *a, long alen) {
    assert(alen == 0);

  const char* how = (const char *)c; // distinguish between normal OK and fallback
  if (strncmp(how,"Init Violation",clen) == 0) {
    printf("IV\n");
    // Initial conditions violated
  } else if (strncmp(how,"Plant Violation",clen) == 0) {
    // Plant monitor violated
      printf("PV\n");
  }  else {
    // Unknown string -- should never occur
    assert(false);
  }
}


void cml_exit(void) {
  longjmp(env,1);
}

int main (int argc, char **argv) {

  /*
   * Insert initialization code here
   */

  // Passing control to CakeML
  int sj = 0;
  sj = setjmp(env);
  if (sj == 0) {
    //    printf("Invoking CML\n");
    int ret = cml_main(argc,argv);
        printf("CML Return value: %d\n", ret);
    // CakeML return value is stored in ret
  } else {
        printf("Abnormal exit: %d\n",sj);
  }

  /*
   * Insert cleanup code here
   */
}
