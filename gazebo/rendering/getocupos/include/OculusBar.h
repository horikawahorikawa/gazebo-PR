#ifndef OCUOCULUSBAR
#define OCUOCULUSBAR
class OculusBar
{
 public:
  int argc;
  char **argv;
  OculusBar();
  virtual ~OculusBar();
  void onOculusReset();
  void onOculusDK2();
  float x  = 0;
  float y  = 0;
  float z  = 1.5;
  float x_old  = 0;
  float y_old  = 0;
  float z_old  = 1.5;
  float rr = 0;
  float rp = 0;
  float ry = 0;
  float qw = 0;
  float qx = 0;
  float qy = 0;
  float qz = 0;



 private:
  static bool isRosInit;
  char choice_in;
  bool reset;
  void rosOn();
};
#endif
