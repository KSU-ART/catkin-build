class PIDloop {
   private:
    double Kp;  /**< Proportional tuning value. */
    double Ki;  /**< Integral tuning value. */
    double Kd;  /**< Derivative tuning value. */
    double derivator;   /**< Accumulates error from past readings. */
    double integrator;  /**< Accumulates error from past readings. */
   //private double iMax;    /**< Current true state estimate. */
   //private double iMin;    /**< Current true state estimate. */
   
    double setPoint;    /**< This is the value the PID controller is trying to hold. */

   /**This constructor takes a tuning value for P, I, and D, and also the set point.
    * It also initializes the derivator and integrator to 0.
    *
    * @param Kp Proportional tuning value.
    * @param Ki Integral tuning value.
    * @param Kd Derivative tuning value.
    * @param setPoint The value the loop is aiming for.
    */
    public:
    PIDloop(double Kp, double Ki, double Kd, double setPoint) {
       this->Kp = Kp;
       this->Ki = Ki;
       this->Kd = Kd;
       this->setPoint = setPoint;
       
       //Just starting, so these should be zero since there is no past error.
       this->derivator = 0;
       this->integrator = 0;
   }

   
   
   
   /** Produces an output for the measurement given. Accounts for past behavior
    * responds based on tuning values.
    *
    * @param z
    * @return
    */
    double step(double z)
   {
       double error = setPoint - z;
       
       double P = Kp * error;
       double D = Kd * (error - derivator);
       derivator = error;
       
       integrator += error;
       
       //Add iMin/iMax?
       
       double I = integrator * Ki;
       
       return P + I + D;
   }
   
   /** This allows you to set or change the set point.
    * Since this can be used to change the set point it has to reset integrator
    * and derivator because they have the accumulated error of the previous set
    * point which is no longer relevant.
    *
    * @param sp
    */
    void setSetPoint(double sp)
   {
       this->setPoint = sp;
       this->integrator = 0;
       this->derivator = 0;
   }
   
   double constrain(double z, double floor, double ceil)
   {
	   if (z < floor)
			z = floor;
	   
	   if (z > ceil)
			z = ceil;
			
			return z;
   }
   
};
