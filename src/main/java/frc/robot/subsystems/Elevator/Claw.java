
public class Claw {
    
    private final Neo motorCLAW;
    
    public Claw(){

    motorCLAW = new Neo(CLAW_TRAP_CAN_ID);
    }

    public void dispense(){
        motorCLAW.setTargetVelocity(1);
    }

    public void GiveBack(){
        motorCLAW.setTargetVelocity(-1);
    }
}
