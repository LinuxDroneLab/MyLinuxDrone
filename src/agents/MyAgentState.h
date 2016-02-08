/*
 * MyAgentState.h
 *
 *  Created on: 23 dic 2015
 *      Author: andrea
 */

#ifndef AGENTS_MYAGENTSTATE_H_
#define AGENTS_MYAGENTSTATE_H_

enum class MyAgentState
	: unsigned short {
		Active, ShuttingDown, Stopping, Stopped, ShutDownComplete
};

#endif /* AGENTS_MYAGENTSTATE_H_ */
