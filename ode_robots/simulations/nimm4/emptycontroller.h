#ifndef __EMPTYCONTROLLER_H
#define __EMPTYCONTROLLER_H


#include <selforg/abstractcontroller.h>



/**
 * Empty robot controller.
 * The controller gets a number of input sensor values each timestep
 *  and has to generate a number of output motor values.
 */
class EmptyController : public AbstractController {
public:

    /// contructor (hint: use $ID$ for revision)
    EmptyController(const std::string& name, const std::string& revision)
    : AbstractController(name, revision){}

  /** initialisation of the controller with the given sensor/ motornumber
      Must be called before use. The random generator is optional.
  */
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
    number_sensors = sensornumber;
    number_motors = motornumber;
  };

  /** @return Number of sensors the controller
      was initialised with or 0 if not initialised */
  virtual int getSensorNumber() const {
    return number_sensors;
  };

  /** @return Number of motors the controller
      was initialised with or 0 if not initialised */
  virtual int getMotorNumber() const {
    return number_motors;
  };

  /** performs one step (includes learning).
      Calculates motor commands from sensor inputs.
      @param sensors sensors inputs scaled to [-1,1]
      @param sensornumber length of the sensor array
      @param motors motors outputs. MUST have enough space for motor values!
      @param motornumber length of the provided motor array
  */
  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber){
    assert(number_sensors == sensornumber);
    assert(number_motors == motornumber);

    // turn right in place

    if (sensors[14] > 0.1){// sensor numbers for red ball (local coordinates!) 14 -> z; 12 -> x; 13 -> y
      motors[0]=  sensors[13]+0.5;
      motors[2]=  sensors[13]+0.5;
    } else{
      motors[0]=  0.5;
      motors[2]=  0.5;
    }

    if (sensors[14] < -0.1){
      motors[1]=  sensors[13]+0.5;
      motors[3]=  sensors[13]+0.5;
    } else{
      motors[1]=  0.5;
      motors[3]=  0.5;
    }


//    // turn right in place
//    motors[0]=  1;
//    motors[1]= -1;
//    motors[2]=  1;
//    motors[3]= -1;

//    // turn left in place
//    motors[0]= -1;
//    motors[1]=  1;
//    motors[2]= -1;
//    motors[3]=  1;

//    //drive straight forward
//    for (int i = 0; i < number_motors; i++){
//      motors[i]=1.0;
//    }



  };

  /** performs one step without learning.
      @see step
  */
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors){

  };

//  /** called in motor babbling phase.
//      the motor values are given (by babbling controller) and
//      this controller can learn the basic relations from observed sensors/motors
//   */
//  virtual void motorBabblingStep(const sensor* , int number_sensors,
//                                 const motor* , int number_motors) {};

  /********* STORABLE INTERFACE ******/
  /// @see Storable
  virtual bool store(FILE* f) const {
    Configurable::print(f,"");
    return true;
  }

  /// @see Storable
  virtual bool restore(FILE* f) {
    Configurable::parse(f);
    return true;
  }


protected:

  int number_sensors;
  int number_motors;

  //paramval strength;
  //paramval offset;
};

#endif
