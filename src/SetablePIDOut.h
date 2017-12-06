/*
 * SetablePIDOut.h
 *
 *  Created on: Nov 13, 2017
 *      Author: hank
 */

#ifndef SRC_SETABLEPIDOUT_H_
#define SRC_SETABLEPIDOUT_H_
#include "WPILib.h"

class SetablePIDOut: public PIDOutput{
public:
	double output;
	SetablePIDOut();
	virtual ~SetablePIDOut();

	void PIDWrite(double output);
};

#endif /* SRC_SETABLEPIDOUT_H_ */
