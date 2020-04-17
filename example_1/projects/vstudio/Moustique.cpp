#include "Moustique.h"

Moustique::Moustique(bool initialization = true) {
	vector<CVector3> vec;
	vec.push_back(CVector3(1, 50, 0));
	vec.push_back(CVector3(50, 0, 0));
	vec.push_back(CVector3(0, -50, 0));
	vec.push_back(CVector3(-50, 0, 0));
	vec.push_back(CVector3(0, 25, 0));
	vec.push_back(CVector3(25, 0, 0));
	vec.push_back(CVector3(0, -25, 0));
	vec.push_back(CVector3(-25, 0, 0));;
	vec.push_back(CVector3(0, 10, 0));
	vec.push_back(CVector3(10, 0, 0));
	vec.push_back(CVector3(0, -10, 0));
	vec.push_back(CVector3(-10, 0, 0));
	vec.push_back(CVector3(0, 0, 0));
	if(initialization)
		init(1, "Wasp.wav", vec);

}

void Moustique::init(float sspeed, char* file, vector<CVector3> newPath)
{
	speed = sspeed;
	soundFileName = file;
	position = newPath.at(0);
	newPath.erase(newPath.begin());
	destination = newPath.at(0);
	newPath.erase(newPath.begin());
	path = newPath;
	deplacementType = "direct";
	pathEnded = false;
}

CVector3 Moustique::getPosition()
{
	return position;
}

void Moustique::setPosition(CVector3 newPosition)
{
	position = newPosition;
}

float Moustique::getSpeed()
{
	return speed;
}

void Moustique::setSpeed(float newSpeed)
{
	speed = newSpeed;
}

CVector3 Moustique::getDestination()
{
	return destination;
}

void Moustique::setDestination(CVector3 newDestination)
{
	destination = newDestination;
}

CVector3 Moustique::getDirection()
{
	CVector3 direction = destination - position;
	return normalize(direction);
}

CVector3 Moustique::accessPosition()
{
	/*
		Appellé par le système audio pour obtenir la position du moustique
		Effectue un déplacement après avoir retourné la position
	*/
	CVector3 vec = getPosition();
	updatePosition();
	return vec;
}

void Moustique::updatePosition()
{
	if (deplacementType == "direct") {
		CVector3 direction = getDirection();
		if (vectorLength(destination - position) < 1) {
			position = destination;
		}
		else {
			direction.x *= speed;
			direction.y *= speed;
			direction.z *= speed;
			position = position + direction;
		}
	}
	if (equalVec(position, destination)) {
		if (path.size() > 1) {
			destination = path.at(0);
			path.erase(path.begin());
		}
		else {
			pathEnded = true;
			std::cout << "End of path"<<endl;
		}
	}
}

float Moustique::vectorLength(CVector3 vec)
{
	return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

CVector3 Moustique::normalize(CVector3 vec)
{
	float length = vectorLength(vec);
	vec.x = vec.x / length;
	vec.y = vec.y / length;
	vec.z = vec.z / length;
	return vec;
}
char* Moustique::getFile()
{
	return soundFileName;
}
void Moustique::setFile(char* file)
{
	soundFileName = file;
}
bool Moustique::equalVec(CVector3 lhs, CVector3 rhs)
{
	return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}

bool Moustique::isEnded()
{
	return pathEnded;
}
