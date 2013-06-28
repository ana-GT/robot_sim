/**
 * @function GainsTuningWidget.cpp
 */
#include "GainsTuningWidget.h"

/**
 * @function GainsTuningWidget
 * @brief Constructor
 */
GainsTuningWidget::GainsTuningWidget( QWidget* _parent ) 
    :QWidget( _parent ) {
    
    // Setup our ui
    setupUi(this);

    // Setup commander
    gtc.init( std::string("http://localhost:11311"),
	      std::string("192.168.1.92"),
	      std::string("gainsTuning") );
}

/**
 * @function ~GainsTuningWidget
 * @brief Destructor
 */
GainsTuningWidget::~GainsTuningWidget() {

}

/**
 * @function 
 * @brief
 */
void GainsTuningWidget::on_pushButton_getGains_Legs_clicked( bool check ) {
    
    std::vector< std::vector<double> > gains;
    gains = gtc.getGains();

    // Put the gains in their line edits
    QString str;

    // RIGHT Leg is selected
    if( this->radioButton_rightLeg->isChecked() == true ) {

	this->lineEdit_HY_Kp->setText( str.setNum(gains[robot_sim::robotState::RHY][0]) );
	this->lineEdit_HY_Kd->setText( str.setNum(gains[robot_sim::robotState::RHY][1]) );
	this->lineEdit_HY_Ki->setText( str.setNum(gains[robot_sim::robotState::RHY][2]) );
	this->lineEdit_HY_Kpv->setText( str.setNum(gains[robot_sim::robotState::RHY][3]) );

	this->lineEdit_HR_Kp->setText( str.setNum(gains[robot_sim::robotState::RHR][0]) );
	this->lineEdit_HR_Kd->setText( str.setNum(gains[robot_sim::robotState::RHR][1]) );
	this->lineEdit_HR_Ki->setText( str.setNum(gains[robot_sim::robotState::RHR][2]) );
	this->lineEdit_HR_Kpv->setText( str.setNum(gains[robot_sim::robotState::RHR][3]) );

	this->lineEdit_HP_Kp->setText( str.setNum(gains[robot_sim::robotState::RHP][0]) );
	this->lineEdit_HP_Kd->setText( str.setNum(gains[robot_sim::robotState::RHP][1]) );
	this->lineEdit_HP_Ki->setText( str.setNum(gains[robot_sim::robotState::RHP][2]) );
	this->lineEdit_HP_Kpv->setText( str.setNum(gains[robot_sim::robotState::RHP][3]) );

	this->lineEdit_KP_Kp->setText( str.setNum(gains[robot_sim::robotState::RKP][0]) );
	this->lineEdit_KP_Kd->setText( str.setNum(gains[robot_sim::robotState::RKP][1]) );
	this->lineEdit_KP_Ki->setText( str.setNum(gains[robot_sim::robotState::RKP][2]) );
	this->lineEdit_KP_Kpv->setText( str.setNum(gains[robot_sim::robotState::RKP][3]) );

	this->lineEdit_AP_Kp->setText( str.setNum(gains[robot_sim::robotState::RAP][0]) );
	this->lineEdit_AP_Kd->setText( str.setNum(gains[robot_sim::robotState::RAP][1]) );
	this->lineEdit_AP_Ki->setText( str.setNum(gains[robot_sim::robotState::RAP][2]) );
	this->lineEdit_AP_Kpv->setText( str.setNum(gains[robot_sim::robotState::RAP][3]) );

	this->lineEdit_AR_Kp->setText( str.setNum(gains[robot_sim::robotState::RAR][0]) );
	this->lineEdit_AR_Kd->setText( str.setNum(gains[robot_sim::robotState::RAR][1]) );
	this->lineEdit_AR_Ki->setText( str.setNum(gains[robot_sim::robotState::RAR][2]) );
	this->lineEdit_AR_Kpv->setText( str.setNum(gains[robot_sim::robotState::RAR][3]) );

    }
    // LEFT Leg or BOTH are selected
    else {
	this->lineEdit_HY_Kp->setText( str.setNum(gains[robot_sim::robotState::LHY][0]) );
	this->lineEdit_HY_Kd->setText( str.setNum(gains[robot_sim::robotState::LHY][1]) );
	this->lineEdit_HY_Ki->setText( str.setNum(gains[robot_sim::robotState::LHY][2]) );
	this->lineEdit_HY_Kpv->setText( str.setNum(gains[robot_sim::robotState::LHY][3]) );

	this->lineEdit_HR_Kp->setText( str.setNum(gains[robot_sim::robotState::LHR][0]) );
	this->lineEdit_HR_Kd->setText( str.setNum(gains[robot_sim::robotState::LHR][1]) );
	this->lineEdit_HR_Ki->setText( str.setNum(gains[robot_sim::robotState::LHR][2]) );
	this->lineEdit_HR_Kpv->setText( str.setNum(gains[robot_sim::robotState::LHR][3]) );

	this->lineEdit_HP_Kp->setText( str.setNum(gains[robot_sim::robotState::LHP][0]) );
	this->lineEdit_HP_Kd->setText( str.setNum(gains[robot_sim::robotState::LHP][1]) );
	this->lineEdit_HP_Ki->setText( str.setNum(gains[robot_sim::robotState::LHP][2]) );
	this->lineEdit_HP_Kpv->setText( str.setNum(gains[robot_sim::robotState::LHP][3]) );

	this->lineEdit_KP_Kp->setText( str.setNum(gains[robot_sim::robotState::LKP][0]) );
	this->lineEdit_KP_Kd->setText( str.setNum(gains[robot_sim::robotState::LKP][1]) );
	this->lineEdit_KP_Ki->setText( str.setNum(gains[robot_sim::robotState::LKP][2]) );
	this->lineEdit_KP_Kpv->setText( str.setNum(gains[robot_sim::robotState::LKP][3]) );

	this->lineEdit_AP_Kp->setText( str.setNum(gains[robot_sim::robotState::LAP][0]) );
	this->lineEdit_AP_Kd->setText( str.setNum(gains[robot_sim::robotState::LAP][1]) );
	this->lineEdit_AP_Ki->setText( str.setNum(gains[robot_sim::robotState::LAP][2]) );
	this->lineEdit_AP_Kpv->setText( str.setNum(gains[robot_sim::robotState::LAP][3]) );

	this->lineEdit_AR_Kp->setText( str.setNum(gains[robot_sim::robotState::LAR][0]) );
	this->lineEdit_AR_Kd->setText( str.setNum(gains[robot_sim::robotState::LAR][1]) );
	this->lineEdit_AR_Ki->setText( str.setNum(gains[robot_sim::robotState::LAR][2]) );
	this->lineEdit_AR_Kpv->setText( str.setNum(gains[robot_sim::robotState::LAR][3]) );
    }
}

/**
 * @function 
 * @brief
 */
void GainsTuningWidget::on_pushButton_setGains_Legs_clicked( bool check ) {

    // Set same as the ones we received
    std::vector< std::vector<double> > gains;
    gains = gtc.getGains();

    // And change the ones we are manipulating in the UI
    // RIGHT Leg is selected
    if( this->radioButton_rightLeg->isChecked() == true ||
	this->radioButton_bothLegs->isChecked() == true ) {

	gains[robot_sim::robotState::RHY][0] = this->lineEdit_HY_Kp->text().toDouble();
	gains[robot_sim::robotState::RHY][1] = this->lineEdit_HY_Kd->text().toDouble();
	gains[robot_sim::robotState::RHY][2] = this->lineEdit_HY_Ki->text().toDouble();
	gains[robot_sim::robotState::RHY][3] = this->lineEdit_HY_Kpv->text().toDouble();

	gains[robot_sim::robotState::RHR][0] = this->lineEdit_HR_Kp->text().toDouble();
	gains[robot_sim::robotState::RHR][1] = this->lineEdit_HR_Kd->text().toDouble();
	gains[robot_sim::robotState::RHR][2] = this->lineEdit_HR_Ki->text().toDouble();
	gains[robot_sim::robotState::RHR][3] = this->lineEdit_HR_Kpv->text().toDouble();

	gains[robot_sim::robotState::RHP][0] = this->lineEdit_HP_Kp->text().toDouble();
	gains[robot_sim::robotState::RHP][1] = this->lineEdit_HP_Kd->text().toDouble();
	gains[robot_sim::robotState::RHP][2] = this->lineEdit_HP_Ki->text().toDouble();
	gains[robot_sim::robotState::RHP][3] = this->lineEdit_HP_Kpv->text().toDouble();

	gains[robot_sim::robotState::RKP][0] = this->lineEdit_KP_Kp->text().toDouble();
	gains[robot_sim::robotState::RKP][1] = this->lineEdit_KP_Kd->text().toDouble();
	gains[robot_sim::robotState::RKP][2] = this->lineEdit_KP_Ki->text().toDouble();
	gains[robot_sim::robotState::RKP][3] = this->lineEdit_KP_Kpv->text().toDouble();

	gains[robot_sim::robotState::RAP][0] = this->lineEdit_AP_Kp->text().toDouble();
	gains[robot_sim::robotState::RAP][1] = this->lineEdit_AP_Kd->text().toDouble();
	gains[robot_sim::robotState::RAP][2] = this->lineEdit_AP_Ki->text().toDouble();
	gains[robot_sim::robotState::RAP][3] = this->lineEdit_AP_Kpv->text().toDouble();

	gains[robot_sim::robotState::RAR][0] = this->lineEdit_AR_Kp->text().toDouble();
	gains[robot_sim::robotState::RAR][1] = this->lineEdit_AR_Kd->text().toDouble();
	gains[robot_sim::robotState::RAR][2] = this->lineEdit_AR_Ki->text().toDouble();
	gains[robot_sim::robotState::RAR][3] = this->lineEdit_AR_Kpv->text().toDouble();

    }
    // LEFT Leg or BOTH are selected
    if( this->radioButton_leftLeg->isChecked() == true ||
	this->radioButton_bothLegs->isChecked() == true ) {

	gains[robot_sim::robotState::LHY][0] = this->lineEdit_HY_Kp->text().toDouble();
	gains[robot_sim::robotState::LHY][1] = this->lineEdit_HY_Kd->text().toDouble();
	gains[robot_sim::robotState::LHY][2] = this->lineEdit_HY_Ki->text().toDouble();
	gains[robot_sim::robotState::LHY][3] = this->lineEdit_HY_Kpv->text().toDouble();

	gains[robot_sim::robotState::LHR][0] = this->lineEdit_HR_Kp->text().toDouble();
	gains[robot_sim::robotState::LHR][1] = this->lineEdit_HR_Kd->text().toDouble();
	gains[robot_sim::robotState::LHR][2] = this->lineEdit_HR_Ki->text().toDouble();
	gains[robot_sim::robotState::LHR][3] = this->lineEdit_HR_Kpv->text().toDouble();

	gains[robot_sim::robotState::LHP][0] = this->lineEdit_HP_Kp->text().toDouble();
	gains[robot_sim::robotState::LHP][1] = this->lineEdit_HP_Kd->text().toDouble();
	gains[robot_sim::robotState::LHP][2] = this->lineEdit_HP_Ki->text().toDouble();
	gains[robot_sim::robotState::LHP][3] = this->lineEdit_HP_Kpv->text().toDouble();

	gains[robot_sim::robotState::LKP][0] = this->lineEdit_KP_Kp->text().toDouble();
	gains[robot_sim::robotState::LKP][1] = this->lineEdit_KP_Kd->text().toDouble();
	gains[robot_sim::robotState::LKP][2] = this->lineEdit_KP_Ki->text().toDouble();
	gains[robot_sim::robotState::LKP][3] = this->lineEdit_KP_Kpv->text().toDouble();

	gains[robot_sim::robotState::LAP][0] = this->lineEdit_AP_Kp->text().toDouble();
	gains[robot_sim::robotState::LAP][1] = this->lineEdit_AP_Kd->text().toDouble();
	gains[robot_sim::robotState::LAP][2] = this->lineEdit_AP_Ki->text().toDouble();
	gains[robot_sim::robotState::LAP][3] = this->lineEdit_AP_Kpv->text().toDouble();

	gains[robot_sim::robotState::LAR][0] = this->lineEdit_AR_Kp->text().toDouble();
	gains[robot_sim::robotState::LAR][1] = this->lineEdit_AR_Kd->text().toDouble();
	gains[robot_sim::robotState::LAR][2] = this->lineEdit_AR_Ki->text().toDouble();
	gains[robot_sim::robotState::LAR][3] = this->lineEdit_AR_Kpv->text().toDouble();

    }

    // Send commands with these gains
    gtc.sendGainCommand( gains ); 

		
    printf("Settled gains \n");
}

/**
 * @function
 * @brief
 */
void GainsTuningWidget::on_pushButton_getGains_Arms_clicked( bool check ) {

    std::vector< std::vector<double> > gains;
    gains = gtc.getGains();

    // Put the gains in their line edits
    QString str;

    // RIGHT Leg is selected
    if( this->radioButton_rightArm->isChecked() == true ) {

	this->lineEdit_SP_Kp->setText( str.setNum(gains[robot_sim::robotState::RSP][0]) );
	this->lineEdit_SP_Kd->setText( str.setNum(gains[robot_sim::robotState::RSP][1]) );
	this->lineEdit_SP_Ki->setText( str.setNum(gains[robot_sim::robotState::RSP][2]) );
	this->lineEdit_SP_Kpv->setText( str.setNum(gains[robot_sim::robotState::RSP][3]) );

	this->lineEdit_SR_Kp->setText( str.setNum(gains[robot_sim::robotState::RSR][0]) );
	this->lineEdit_SR_Kd->setText( str.setNum(gains[robot_sim::robotState::RSR][1]) );
	this->lineEdit_SR_Ki->setText( str.setNum(gains[robot_sim::robotState::RSR][2]) );
	this->lineEdit_SR_Kpv->setText( str.setNum(gains[robot_sim::robotState::RSR][3]) );

	this->lineEdit_SY_Kp->setText( str.setNum(gains[robot_sim::robotState::RSY][0]) );
	this->lineEdit_SY_Kd->setText( str.setNum(gains[robot_sim::robotState::RSY][1]) );
	this->lineEdit_SY_Ki->setText( str.setNum(gains[robot_sim::robotState::RSY][2]) );
	this->lineEdit_SY_Kpv->setText( str.setNum(gains[robot_sim::robotState::RSY][3]) );

	this->lineEdit_EP_Kp->setText( str.setNum(gains[robot_sim::robotState::REP][0]) );
	this->lineEdit_EP_Kd->setText( str.setNum(gains[robot_sim::robotState::REP][1]) );
	this->lineEdit_EP_Ki->setText( str.setNum(gains[robot_sim::robotState::REP][2]) );
	this->lineEdit_EP_Kpv->setText( str.setNum(gains[robot_sim::robotState::REP][3]) );

	this->lineEdit_WY_Kp->setText( str.setNum(gains[robot_sim::robotState::RWY][0]) );
	this->lineEdit_WY_Kd->setText( str.setNum(gains[robot_sim::robotState::RWY][1]) );
	this->lineEdit_WY_Ki->setText( str.setNum(gains[robot_sim::robotState::RWY][2]) );
	this->lineEdit_WY_Kpv->setText( str.setNum(gains[robot_sim::robotState::RWY][3]) );

	this->lineEdit_WP_Kp->setText( str.setNum(gains[robot_sim::robotState::RWP][0]) );
	this->lineEdit_WP_Kd->setText( str.setNum(gains[robot_sim::robotState::RWP][1]) );
	this->lineEdit_WP_Ki->setText( str.setNum(gains[robot_sim::robotState::RWP][2]) );
	this->lineEdit_WP_Kpv->setText( str.setNum(gains[robot_sim::robotState::RWP][3]) );

	this->lineEdit_WR_Kp->setText( str.setNum(gains[robot_sim::robotState::RWR][0]) );
	this->lineEdit_WR_Kd->setText( str.setNum(gains[robot_sim::robotState::RWR][1]) );
	this->lineEdit_WR_Ki->setText( str.setNum(gains[robot_sim::robotState::RWR][2]) );
	this->lineEdit_WR_Kpv->setText( str.setNum(gains[robot_sim::robotState::RWR][3]) );


    }
    // LEFT Leg or BOTH are selected
    else {
	this->lineEdit_SP_Kp->setText( str.setNum(gains[robot_sim::robotState::LSP][0]) );
	this->lineEdit_SP_Kd->setText( str.setNum(gains[robot_sim::robotState::LSP][1]) );
	this->lineEdit_SP_Ki->setText( str.setNum(gains[robot_sim::robotState::LSP][2]) );
	this->lineEdit_SP_Kpv->setText( str.setNum(gains[robot_sim::robotState::LSP][3]) );

	this->lineEdit_SR_Kp->setText( str.setNum(gains[robot_sim::robotState::LSR][0]) );
	this->lineEdit_SR_Kd->setText( str.setNum(gains[robot_sim::robotState::LSR][1]) );
	this->lineEdit_SR_Ki->setText( str.setNum(gains[robot_sim::robotState::LSR][2]) );
	this->lineEdit_SR_Kpv->setText( str.setNum(gains[robot_sim::robotState::LSR][3]) );

	this->lineEdit_SY_Kp->setText( str.setNum(gains[robot_sim::robotState::LSY][0]) );
	this->lineEdit_SY_Kd->setText( str.setNum(gains[robot_sim::robotState::LSY][1]) );
	this->lineEdit_SY_Ki->setText( str.setNum(gains[robot_sim::robotState::LSY][2]) );
	this->lineEdit_SY_Kpv->setText( str.setNum(gains[robot_sim::robotState::LSY][3]) );

	this->lineEdit_EP_Kp->setText( str.setNum(gains[robot_sim::robotState::LEP][0]) );
	this->lineEdit_EP_Kd->setText( str.setNum(gains[robot_sim::robotState::LEP][1]) );
	this->lineEdit_EP_Ki->setText( str.setNum(gains[robot_sim::robotState::LEP][2]) );
	this->lineEdit_EP_Kpv->setText( str.setNum(gains[robot_sim::robotState::LEP][3]) );

	this->lineEdit_WY_Kp->setText( str.setNum(gains[robot_sim::robotState::LWY][0]) );
	this->lineEdit_WY_Kd->setText( str.setNum(gains[robot_sim::robotState::LWY][1]) );
	this->lineEdit_WY_Ki->setText( str.setNum(gains[robot_sim::robotState::LWY][2]) );
	this->lineEdit_WY_Kpv->setText( str.setNum(gains[robot_sim::robotState::LWY][3]) );

	this->lineEdit_WP_Kp->setText( str.setNum(gains[robot_sim::robotState::LWP][0]) );
	this->lineEdit_WP_Kd->setText( str.setNum(gains[robot_sim::robotState::LWP][1]) );
	this->lineEdit_WP_Ki->setText( str.setNum(gains[robot_sim::robotState::LWP][2]) );
	this->lineEdit_WP_Kpv->setText( str.setNum(gains[robot_sim::robotState::LWP][3]) );

	this->lineEdit_WR_Kp->setText( str.setNum(gains[robot_sim::robotState::LWR][0]) );
	this->lineEdit_WR_Kd->setText( str.setNum(gains[robot_sim::robotState::LWR][1]) );
	this->lineEdit_WR_Ki->setText( str.setNum(gains[robot_sim::robotState::LWR][2]) );
	this->lineEdit_WR_Kpv->setText( str.setNum(gains[robot_sim::robotState::LWR][3]) );
    }

}

/**
 *
 */
void GainsTuningWidget::on_pushButton_setGains_Arms_clicked( bool check ) {

    // Set same as the ones we received
    std::vector< std::vector<double> > gains;
    gains = gtc.getGains();

    // And change the ones we are manipulating in the UI
    // RIGHT Leg is selected
    if( this->radioButton_rightArm->isChecked() == true ||
	this->radioButton_bothArms->isChecked() == true ) {

	gains[robot_sim::robotState::RSP][0] = this->lineEdit_SP_Kp->text().toDouble();
	gains[robot_sim::robotState::RSP][1] = this->lineEdit_SP_Kd->text().toDouble();
	gains[robot_sim::robotState::RSP][2] = this->lineEdit_SP_Ki->text().toDouble();
	gains[robot_sim::robotState::RSP][3] = this->lineEdit_SP_Kpv->text().toDouble();

	gains[robot_sim::robotState::RSR][0] = this->lineEdit_SR_Kp->text().toDouble();
	gains[robot_sim::robotState::RSR][1] = this->lineEdit_SR_Kd->text().toDouble();
	gains[robot_sim::robotState::RSR][2] = this->lineEdit_SR_Ki->text().toDouble();
	gains[robot_sim::robotState::RSR][3] = this->lineEdit_SR_Kpv->text().toDouble();

	gains[robot_sim::robotState::RSY][0] = this->lineEdit_SY_Kp->text().toDouble();
	gains[robot_sim::robotState::RSY][1] = this->lineEdit_SY_Kd->text().toDouble();
	gains[robot_sim::robotState::RSY][2] = this->lineEdit_SY_Ki->text().toDouble();
	gains[robot_sim::robotState::RSY][3] = this->lineEdit_SY_Kpv->text().toDouble();

	gains[robot_sim::robotState::REP][0] = this->lineEdit_EP_Kp->text().toDouble();
	gains[robot_sim::robotState::REP][1] = this->lineEdit_EP_Kd->text().toDouble();
	gains[robot_sim::robotState::REP][2] = this->lineEdit_EP_Ki->text().toDouble();
	gains[robot_sim::robotState::REP][3] = this->lineEdit_EP_Kpv->text().toDouble();

	gains[robot_sim::robotState::RWY][0] = this->lineEdit_WY_Kp->text().toDouble();
	gains[robot_sim::robotState::RWY][1] = this->lineEdit_WY_Kd->text().toDouble();
	gains[robot_sim::robotState::RWY][2] = this->lineEdit_WY_Ki->text().toDouble();
	gains[robot_sim::robotState::RWY][3] = this->lineEdit_WY_Kpv->text().toDouble();

	gains[robot_sim::robotState::RWP][0] = this->lineEdit_WP_Kp->text().toDouble();
	gains[robot_sim::robotState::RWP][1] = this->lineEdit_WP_Kd->text().toDouble();
	gains[robot_sim::robotState::RWP][2] = this->lineEdit_WP_Ki->text().toDouble();
	gains[robot_sim::robotState::RWP][3] = this->lineEdit_WP_Kpv->text().toDouble();

	gains[robot_sim::robotState::RWR][0] = this->lineEdit_WR_Kp->text().toDouble();
	gains[robot_sim::robotState::RWR][1] = this->lineEdit_WR_Kd->text().toDouble();
	gains[robot_sim::robotState::RWR][2] = this->lineEdit_WR_Ki->text().toDouble();
	gains[robot_sim::robotState::RWR][3] = this->lineEdit_WR_Kpv->text().toDouble();

    }
    // LEFT Leg or BOTH are selected
    if( this->radioButton_leftArm->isChecked() == true ||
	this->radioButton_bothArms->isChecked() == true ) {

	gains[robot_sim::robotState::LSP][0] = this->lineEdit_SP_Kp->text().toDouble();
	gains[robot_sim::robotState::LSP][1] = this->lineEdit_SP_Kd->text().toDouble();
	gains[robot_sim::robotState::LSP][2] = this->lineEdit_SP_Ki->text().toDouble();
	gains[robot_sim::robotState::LSP][3] = this->lineEdit_SP_Kpv->text().toDouble();

	gains[robot_sim::robotState::LSR][0] = this->lineEdit_SR_Kp->text().toDouble();
	gains[robot_sim::robotState::LSR][1] = this->lineEdit_SR_Kd->text().toDouble();
	gains[robot_sim::robotState::LSR][2] = this->lineEdit_SR_Ki->text().toDouble();
	gains[robot_sim::robotState::LSR][3] = this->lineEdit_SR_Kpv->text().toDouble();

	gains[robot_sim::robotState::LSY][0] = this->lineEdit_SY_Kp->text().toDouble();
	gains[robot_sim::robotState::LSY][1] = this->lineEdit_SY_Kd->text().toDouble();
	gains[robot_sim::robotState::LSY][2] = this->lineEdit_SY_Ki->text().toDouble();
	gains[robot_sim::robotState::LSY][3] = this->lineEdit_SY_Kpv->text().toDouble();

	gains[robot_sim::robotState::LEP][0] = this->lineEdit_EP_Kp->text().toDouble();
	gains[robot_sim::robotState::LEP][1] = this->lineEdit_EP_Kd->text().toDouble();
	gains[robot_sim::robotState::LEP][2] = this->lineEdit_EP_Ki->text().toDouble();
	gains[robot_sim::robotState::LEP][3] = this->lineEdit_EP_Kpv->text().toDouble();

	gains[robot_sim::robotState::LWY][0] = this->lineEdit_WY_Kp->text().toDouble();
	gains[robot_sim::robotState::LWY][1] = this->lineEdit_WY_Kd->text().toDouble();
	gains[robot_sim::robotState::LWY][2] = this->lineEdit_WY_Ki->text().toDouble();
	gains[robot_sim::robotState::LWY][3] = this->lineEdit_WY_Kpv->text().toDouble();

	gains[robot_sim::robotState::LWP][0] = this->lineEdit_WP_Kp->text().toDouble();
	gains[robot_sim::robotState::LWP][1] = this->lineEdit_WP_Kd->text().toDouble();
	gains[robot_sim::robotState::LWP][2] = this->lineEdit_WP_Ki->text().toDouble();
	gains[robot_sim::robotState::LWP][3] = this->lineEdit_WP_Kpv->text().toDouble();

	gains[robot_sim::robotState::LWR][0] = this->lineEdit_WR_Kp->text().toDouble();
	gains[robot_sim::robotState::LWR][1] = this->lineEdit_WR_Kd->text().toDouble();
	gains[robot_sim::robotState::LWR][2] = this->lineEdit_WR_Ki->text().toDouble();
	gains[robot_sim::robotState::LWR][3] = this->lineEdit_WR_Kpv->text().toDouble();

    }

    // Send commands with these gains
    gtc.sendGainCommand( gains ); 

		
    printf("Settled gains Arms \n");


}

void GainsTuningWidget::on_pushButton_getGains_TorsoNeck_clicked( bool check ) {

    std::vector< std::vector<double> > gains;
    gains = gtc.getGains();

    // Put the gains in their line edits
    QString str;


    this->lineEdit_TSY_Kp->setText( str.setNum(gains[robot_sim::robotState::TSY][0]) );
    this->lineEdit_TSY_Kd->setText( str.setNum(gains[robot_sim::robotState::TSY][1]) );
    this->lineEdit_TSY_Ki->setText( str.setNum(gains[robot_sim::robotState::TSY][2]) );
    this->lineEdit_TSY_Kpv->setText( str.setNum(gains[robot_sim::robotState::TSY][3]) );

    this->lineEdit_NKY_Kp->setText( str.setNum(gains[robot_sim::robotState::NKY][0]) );
    this->lineEdit_NKY_Kd->setText( str.setNum(gains[robot_sim::robotState::NKY][1]) );
    this->lineEdit_NKY_Ki->setText( str.setNum(gains[robot_sim::robotState::NKY][2]) );
    this->lineEdit_NKY_Kpv->setText( str.setNum(gains[robot_sim::robotState::NKY][3]) );

    this->lineEdit_NKP_Kp->setText( str.setNum(gains[robot_sim::robotState::NKP][0]) );
    this->lineEdit_NKP_Kd->setText( str.setNum(gains[robot_sim::robotState::NKP][1]) );
    this->lineEdit_NKP_Ki->setText( str.setNum(gains[robot_sim::robotState::NKP][2]) );
    this->lineEdit_NKP_Kpv->setText( str.setNum(gains[robot_sim::robotState::NKP][3]) );

}


/**
 * @function 
 * @brief
 */
void GainsTuningWidget::on_pushButton_setGains_TorsoNeck_clicked( bool check ) {

    // Set same as the ones we received
    std::vector< std::vector<double> > gains;
    gains = gtc.getGains();
    
    gains[robot_sim::robotState::TSY][0] = this->lineEdit_TSY_Kp->text().toDouble();
    gains[robot_sim::robotState::TSY][1] = this->lineEdit_TSY_Kd->text().toDouble();
    gains[robot_sim::robotState::TSY][2] = this->lineEdit_TSY_Ki->text().toDouble();
    gains[robot_sim::robotState::TSY][3] = this->lineEdit_TSY_Kpv->text().toDouble();
    
    gains[robot_sim::robotState::NKY][0] = this->lineEdit_NKY_Kp->text().toDouble();
    gains[robot_sim::robotState::NKY][1] = this->lineEdit_NKY_Kd->text().toDouble();
    gains[robot_sim::robotState::NKY][2] = this->lineEdit_NKY_Ki->text().toDouble();
    gains[robot_sim::robotState::NKY][3] = this->lineEdit_NKY_Kpv->text().toDouble();
    
    gains[robot_sim::robotState::NKP][0] = this->lineEdit_NKP_Kp->text().toDouble();
    gains[robot_sim::robotState::NKP][1] = this->lineEdit_NKP_Kd->text().toDouble();
    gains[robot_sim::robotState::NKP][2] = this->lineEdit_NKP_Ki->text().toDouble();
    gains[robot_sim::robotState::NKP][3] = this->lineEdit_NKP_Kpv->text().toDouble();
    

    // Send commands with these gains
    gtc.sendGainCommand( gains ); 
    
    
    printf("Settled gains Torso Neck \n");

}


