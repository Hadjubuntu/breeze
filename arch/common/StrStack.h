/*
 * StrStack.h
 *
 *  Created on: Jan 28, 2015
 *      Author: adrien
 */

#ifndef STRSTACK_H_
#define STRSTACK_H_

#define STACK_SIZE 20
#define PACK_LENGTH_STR 100


class StrStack {
private:
	bool stackIsReady = false;
	int currentIdx;
	int makeIdx;
	bool nextRound ;
	char stack[STACK_SIZE][PACK_LENGTH_STR];

public:
	void init() ;
	void insert(char *pStr);
	void make(void (*callbackFunction)(int));
	char* get(int);
};


void StrStack::init() {
	currentIdx = 0;
	stackIsReady = true;
}

char* StrStack::get(int pIndex) {
	return stack[pIndex];
}

void StrStack::make(void (*callbackFunction)(int)) {
	int endPosition = 0;
	if (nextRound) {
		endPosition = STACK_SIZE;
	}
	else {
		endPosition = currentIdx;
	}

	for (int k = makeIdx ; k < endPosition; k ++) {
		callbackFunction(k);
	}
	makeIdx = endPosition;


	if (nextRound) {
		for (int l = 0; l < currentIdx; l ++) {
			callbackFunction(l);
		}
		nextRound = false;
		makeIdx = currentIdx;
	}
}

void StrStack::insert(char *pStr) {
	if (stackIsReady) {
		if (strlen(pStr) < PACK_LENGTH_STR) {
			strncpy(stack[currentIdx], pStr, strlen(pStr));

			currentIdx ++;
			if (currentIdx >= STACK_SIZE) {
				currentIdx = 0;
				nextRound = true;
			}
		}
	}
}


#endif /* STRSTACK_H_ */
