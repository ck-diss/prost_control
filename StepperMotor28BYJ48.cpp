#include <iostream>
#include <ctime>
#include <stdlib.h>
#include "StepperMotor28BYJ48.h"
#include <qtconcurrentrun.h>

using namespace std;
QFuture<void> t_motor;
QFuture<void> t_motor2;
int countSteps;

const bool StepperMotor28BYJ48::WAVE_DRIVE_MOTOR_SEQUENCE[][4] = {
		{ LOW,  LOW,  LOW,  HIGH },
		{ LOW,  LOW,  HIGH, LOW },
		{ LOW,  HIGH, LOW,  LOW },
		{ HIGH, LOW,  LOW,  LOW },
		{ LOW,  LOW,  LOW,  HIGH },
		{ LOW,  LOW,  HIGH, LOW },
		{ LOW,  HIGH, LOW,  LOW },
		{ HIGH, LOW,  LOW,  LOW }
};

const bool StepperMotor28BYJ48::FULL_STEP_MOTOR_SEQUENCE[][4] = {
		{ LOW,  LOW,  LOW,  HIGH },
		{ LOW,  LOW,  HIGH, LOW },
		{ LOW,  HIGH, LOW,  LOW },
		{ HIGH, LOW,  LOW,  LOW },
		{ LOW,  LOW,  LOW,  HIGH },
		{ LOW,  LOW,  HIGH, LOW },
		{ LOW,  HIGH, LOW,  LOW },
		{ HIGH, LOW,  LOW,  LOW }
};

const bool StepperMotor28BYJ48::HALF_STEP_MOTOR_SEQUENCE[][4] = {
		{ LOW,  LOW,  LOW,  HIGH },
		{ LOW,  LOW,  HIGH, HIGH },
		{ LOW,  LOW,  HIGH, LOW },
		{ LOW,  HIGH, HIGH, LOW },
		{ LOW,  HIGH, LOW,  LOW },
		{ HIGH, HIGH, LOW,  LOW },
		{ HIGH, LOW,  LOW,  LOW },
		{ HIGH, LOW,  LOW,  HIGH }
};
StepperMotor28BYJ48::StepperMotor28BYJ48() {}
void StepperMotor28BYJ48::init(const unsigned int pinA, const unsigned int pinB, const unsigned int pinC,
		const unsigned int pinD, const unsigned int stepDuration, const SteppingMethod steppingMethod)
{
	inputPins[0] = pinA;
	inputPins[1] = pinB;
	inputPins[2] = pinC;
	inputPins[3] = pinD;

	for (int i = 0; i < 4; i++) {
        bcm2835_gpio_fsel(inputPins[i], BCM2835_GPIO_FSEL_OUTP);
        //wPI: pinMode(inputPins[i], OUTPUT);
    }

	this->stepDuration = stepDuration;
	this->steppingMethod = steppingMethod;

	started = false;
    countSteps = 0;
}

void StepperMotor28BYJ48::fullRotation(const int noOfRotations) {
	halfRotation(2*noOfRotations);
}

void StepperMotor28BYJ48::halfRotation(const int noOfHalfRotations) {
	quarterRotation(2*noOfHalfRotations);
}

void StepperMotor28BYJ48::quarterRotation(const int noOfQuarterRotations) {
	switch (steppingMethod) {
		case HALF_STEP:
			step(2 * 512 * noOfQuarterRotations);
			break;
		default:
			step(512 * noOfQuarterRotations);
			break;
	}
}

void StepperMotor28BYJ48::angleRotation(const float angle) {
	int steps;
	switch (steppingMethod) {
		case HALF_STEP:
			steps = (int)(512*8*angle)/360;
			break;
		default:
			steps = (int)(512*4*angle)/360;
			break;
	}
	step(steps);
}

void StepperMotor28BYJ48::setStepDurartion(const unsigned int stepDuration) {
	this->stepDuration = stepDuration;
}

void StepperMotor28BYJ48::setSteppingMethod(const SteppingMethod steppingMethod) {
	this->steppingMethod = steppingMethod;
}

void StepperMotor28BYJ48::step(const int noOfSteps) {
    countSteps+=noOfSteps;
    //qDebug() << countSteps;

    if (countSteps > 0) {
        if (noOfSteps > 0) {
            for (int currentStep = noOfSteps; currentStep > 0; currentStep--) {
                int currentSequenceNo = currentStep % 8;
                writeSequence(currentSequenceNo);
            }
        } else {
            for (int currentStep = 0; currentStep < abs(noOfSteps); currentStep++) {
                int currentSequenceNo = currentStep % 8;
                writeSequence(currentSequenceNo);
            }
        }
    }
}

void StepperMotor28BYJ48::writeSequence(const unsigned int sequenceNo) {
	for (int i = 0; i < 4; i++) {
		switch(steppingMethod) {
			case WAVE_DRIVE:
                bcm2835_gpio_write(inputPins[i], WAVE_DRIVE_MOTOR_SEQUENCE[sequenceNo][i]);
                //wPI: digitalWrite(inputPins[i], WAVE_DRIVE_MOTOR_SEQUENCE[sequenceNo][i]);
                break;
			case FULL_STEP:
                bcm2835_gpio_write(inputPins[i], FULL_STEP_MOTOR_SEQUENCE[sequenceNo][i]);
				break;
			default:
                bcm2835_gpio_write(inputPins[i], HALF_STEP_MOTOR_SEQUENCE[sequenceNo][i]);
				break;
		}
	}
    delay(stepDuration);
	for (int i = 0; i < 4; i++) {
        bcm2835_gpio_write(inputPins[i], LOW);
	}
}

void StepperMotor28BYJ48::startClockwise() {
    if (started) {
        stop();
    }
    started = true;
    //editmotorThread = new thread(&StepperMotor28BYJ48::start, this, 5);
    t_motor = QtConcurrent::run(this, &StepperMotor28BYJ48::start, 5);
}

void StepperMotor28BYJ48::startCounterClockwise() {
    if (started) {
        stop();
    }
    started = true;

    //motorThread = new thread(&StepperMotor28BYJ48::start, this, -5);
    if (countSteps>0)
        t_motor = QtConcurrent::run(this, &StepperMotor28BYJ48::start, -5);
    else
        stop();
}

void StepperMotor28BYJ48::start(const int direction) {
	while (started) {
		step(direction);
	}
}

void StepperMotor28BYJ48::stop() {
    if (started) {
        started = false;
        //edit motorThread->join();
        //edit delete(motorThread);
    }
    if (t_motor.isRunning()) {
        t_motor.waitForFinished();
    }

}

int StepperMotor28BYJ48::getCountSteps() {
    return countSteps;
}

bool StepperMotor28BYJ48::isStarted() {
	return started;
}

void StepperMotor28BYJ48::performDemo() {
	clock_t begin, end;

	cout << "Full rotation clockwise in wave drive method... " << endl;
	begin = clock();
	setSteppingMethod(WAVE_DRIVE);
	fullRotation(1);
	end = clock();
	cout << "(took " << (end - begin) << ")" << endl;

	cout << "Full rotation counterclockwise in full step method... " << endl;
	begin = clock();
	setSteppingMethod(FULL_STEP);
	fullRotation(-1);
	end = clock();
	cout << "(took " << (end - begin) << ")" << endl;

	cout << "Full rotation clockwise in half step method... " << endl;
	begin = clock();
	setSteppingMethod(HALF_STEP);
	fullRotation(1);
	end = clock();
	cout << "(took " << (end - begin) << ")" << endl;

	cout << "Half rotation counterclockwise in full step method... " << endl;
	begin = clock();
	setSteppingMethod(FULL_STEP);
	halfRotation(-1);
	end = clock();
	cout << "(took " << (end - begin) << ")" << endl;

	cout << "Quarter rotation clockwise in full step method... " << endl;
	begin = clock();
	setSteppingMethod(FULL_STEP);
	quarterRotation(1);
	end = clock();
	cout << "(took " << (end - begin) << ")" << endl;

	cout << "180 degree rotation counterclockwise in full step method... " << endl;
	begin = clock();
	setSteppingMethod(FULL_STEP);
	angleRotation(-180);
	end = clock();
	cout << "(took " << (end - begin) << ")" << endl;

	cout << "90 degree rotation clockwise in half step method... " << endl;
	begin = clock();
	setSteppingMethod(HALF_STEP);
	angleRotation(270);
	end = clock();
	cout << "(took " << (end - begin) << ")" << endl;
}
