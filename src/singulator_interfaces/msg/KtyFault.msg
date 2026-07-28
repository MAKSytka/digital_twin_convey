uint8 INFO=0
uint8 WARNING=1
uint8 CRITICAL=2

string PRODUCT_OUTSIDE_KTY=product_outside_kty
string PRODUCT_JAMMED_ON_CHUTE=product_jammed_on_chute
string KTY_NOT_INSTALLED=kty_not_installed
string MASS_EXCEEDED=mass_exceeded
string CAMERA_LOST_VIEW=camera_lost_view
string PRODUCT_STILL_MOVING=product_still_moving

std_msgs/Header header
uint32 cycle_id
uint8 severity
string code
string details
bool active
