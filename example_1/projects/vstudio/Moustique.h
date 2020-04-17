#pragma once
#include <string>
#include <math.h>
#include <Common\Vector3.h>
#include <vector>

using namespace std;
using namespace Common;

class Moustique
{
	CVector3					position;
	float						speed;
	CVector3					destination;
	char*						soundFileName = "Wasp.wav";
	string						deplacementType = "direct";
	vector<CVector3>			path;
	bool						pathEnded;
public :
	Moustique(bool initialization);
	void init(float sspeed, char* file, vector<CVector3> newPath);
	CVector3 getPosition();
	void setPosition(CVector3 newPosition);

	float getSpeed();
	void setSpeed(float newSpeed);
	
	CVector3 getDestination();
	void setDestination(CVector3 newDestination);

	CVector3 getDirection();

	CVector3 accessPosition();
	void updatePosition();

	float vectorLength(CVector3 vec);
	CVector3 normalize(CVector3 vec);

	char* getFile();
	void setFile(char* file);

	bool equalVec(CVector3 lhs, CVector3 rhs);
	bool isEnded();
};

