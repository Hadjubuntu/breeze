/*
 * StrUtils.h
 *
 *  Created on: Jan 28, 2015
 *      Author: adrien
 */

#ifndef STRUTILS_H_
#define STRUTILS_H_


#include <stdio.h>
#include <string.h>
#include <stdlib.h>


/**
 * Takes any char array, and reset each character
 * so that strcat will start from beginning of the string
 */
void str_resetCharArray(char *p) {
	int nbBytes = strlen(p);
	memset(&p[0], 0, nbBytes);
}

/**
 * Test whether a string starts with another string
 */
bool str_startsWith(char *pStr, char *pStart) {
	return (strlen(pStart) <= strlen(pStr) && ( strncmp(pStart,pStr,strlen(pStart)) == 0));
}

/**
 * Tells whether a string ends with
 */
bool str_endWith(char *pStr, char *pEnd) {
	bool endwidth = true;
	int sizeStr = strlen(pStr);
	int sizeEnd = strlen(pEnd);

	if (sizeEnd < sizeStr) {
		int i_start = sizeStr-sizeEnd;
		for (int i=i_start; i < sizeStr; i ++) {
			if (pStr[i] != pEnd[i-i_start]) {
				endwidth = false;
			}
		}
	}
	else {
		endwidth = false;
	}

	return endwidth;
}

/**
 * Count how many 'pChar' appears in a string
 */
int str_countChar(char *pStr, char pChar) {
	int count = 0;
	int sizeStr = strlen(pStr);

	for (int i = 0; i < sizeStr; i ++) {
		if (pStr[i] == pChar) {
			count ++;
		}
	}

	return count;
}

/**
 * Split string and extract the part between start and end
 */
char *str_substring(char *pStr, int start, int end) {
	char *output;
	int lengthStr = strlen(pStr) ;

	if (start <= lengthStr && end <= lengthStr) {
		output = (char*) malloc((end-start)*sizeof(char));
		for (int i = 0; i < end-start; i ++) {
			output[i] = pStr[start + i];
		}
	}

	return output;
}

/**
 * Return boolean if two strings are equals
 */
bool str_equals(char *str1, char *str2) {
	int cmpStr = strcmp(str1, str2) ;
	return (cmpStr == 0) ;
}

/**
 * Struct of a string array
 */
typedef struct T_STRING_ARRAY {
	char **array;
	int sizeArray;
} StringArray;

/**
 * Explode a string into array using a delimiter char value
 */
StringArray str_explode(char *pStr, char limiter) {

	char source[strlen(pStr) + 1];
	strcpy(source, pStr);

	char str_limiter[1];
	str_limiter[0] = limiter;

	if (str_endWith(pStr, str_limiter)) {
		strcat(source, "\n");
	}

	StringArray outputSA;

	int sizeOutput = str_countChar(source, limiter);


	/* allocation mémoire pour le tableau de sous-tableaux :   */
	char **output = (char**) malloc((sizeOutput+1) * sizeof(char*));


	char * pch;
	int lengthToCopy = 0;
	pch = strtok (source, "|");

	for (int iPos = 0; iPos <= sizeOutput; iPos++) {
		lengthToCopy = strlen(pch);
		if (pch[lengthToCopy-1] == '\n') {
			lengthToCopy --;
		}

		output[iPos] = (char *) malloc((lengthToCopy+1) * sizeof(char));
		strncpy(output[iPos], pch, lengthToCopy);
		pch = strtok (NULL, "|");
	}

	outputSA.array = output;
	outputSA.sizeArray = sizeOutput;

	free(pch);

	return outputSA;
}


/**
 * Struct of a string array
 */
typedef struct T_EMBEDDED_STRING_ARRAY {
	char array[20][125];
	int sizeArray;
} EStringArray;


// Embedded version
EStringArray _embedded_str_explode(char *pStr, char limiter) {

	char source[strlen(pStr) + 1];
	strcpy(source, pStr);

	char str_limiter[1];
	str_limiter[0] = limiter;

	if (str_endWith(pStr, str_limiter)) {
		strcat(source, "\n");
	}

	EStringArray outputSA;

	int sizeOutput = str_countChar(source, limiter);


	/* allocation mémoire pour le tableau de sous-tableaux :   */



	char * pch;
	int lengthToCopy = 0;
	pch = strtok (source, "|");

	for (int iPos = 0; iPos <= sizeOutput; iPos++) {
		lengthToCopy = strlen(pch);
		if (lengthToCopy > 0 && pch[0] != '\n') {
			if (pch[lengthToCopy-1] == '\n') {
				lengthToCopy --;
			}

			str_resetCharArray(outputSA.array[iPos]);
			strncpy(outputSA.array[iPos], pch, lengthToCopy);
		}
		pch = strtok (NULL, "|");
	}

	outputSA.sizeArray = sizeOutput;

	free(pch);


	return outputSA;
}


#endif /* STRUTILS_H_ */
