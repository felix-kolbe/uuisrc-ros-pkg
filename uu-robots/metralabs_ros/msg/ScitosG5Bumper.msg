# Message used to publish the bumper state of Scitos G5 robots.
Header header

# Whether the bumper is currently pressed.
bool bumper_pressed

# Whether internal motor_stop is active and disables the robots drive motors.
# The motor_stop is activated on a bumper hit and can be set manually.
# The motor_stop can be deactivated via topic or set via service.
bool motor_stop
