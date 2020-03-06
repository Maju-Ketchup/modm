/**
*  Copyright (c) 2018, Marten Junga (Github.com/Maju-Ketchup)
* All Rights Reserved.
*
* The file is part of the modm library and is released under the 3-clause BSD
* license. See the file `LICENSE` for the full license governing this code.
*
*
* The headder contains the class implementation of the IEEE standart 802.15.4-2011 Frame
* current max size is 255 bytes but some devices are able to send 1023 bytes
* Set always control first
*
*/
#include "multilateration.hpp"
#include <modm/math.hpp>
#include <modm/debug/logger.hpp>

using namespace modm::math;

void modm::multilateration::activemultilaterationNewton(Vector<floatunit, 3> &output,
														Vector<floatunit, 3> anchor1,
														Vector<floatunit, 3> anchor2,
														Vector<floatunit, 3> anchor3,
														Vector<floatunit, 3> anchor4,
														floatunit receiveAnchor1,
														floatunit receiveAnchor2,
														floatunit receiveAnchor3,
														floatunit receiveAnchor4)
{
	floatunit speedoflight = 299792548.f;
	//Berechne TOF zum Ausgangsvektor
    floatunit tof0 = powl(anchor1[0] - output[0],2) + powl(anchor1[1] - output[1],2) + powl(anchor1[2] - output[2],2);
	tof0 = (sqrt(tof0)/speedoflight)-receiveAnchor1;
    floatunit tof1 = powl(anchor2[0] - output[0],2) + powl(anchor2[1] - output[1],2) + powl(anchor2[2] - output[2],2);
	tof1 = (sqrt(tof1)/speedoflight)-receiveAnchor2;
    floatunit tof2 = powl(anchor3[0] - output[0],2) + powl(anchor3[1] - output[1],2) + powl(anchor3[2] - output[2],2);
	tof2 = (sqrt(tof2)/speedoflight)-receiveAnchor3;
    floatunit tof3 = powl(anchor4[0] - output[0],2) + powl(anchor4[1] - output[1],2) + powl(anchor4[2] - output[2],2);
	tof3 = (sqrt(tof3)/speedoflight)-receiveAnchor4;
	modm::Matrix<floatunit,4,1> result;
	result[0][0]=output[0];
	result[1][0]=output[1];
	result[2][0]=output[2];
    result[3][0]=(-1)*((tof0+tof1+tof2+tof3)/4);
	modm::Matrix<floatunit,4,4> anchormatrix;
	anchormatrix[0][0] = anchor1[0];
	anchormatrix[0][1] = anchor1[1];
	anchormatrix[0][2] = anchor1[2];
	anchormatrix[0][3] = receiveAnchor1;

	anchormatrix[1][0] = anchor2[0];
	anchormatrix[1][1] = anchor2[1];
	anchormatrix[1][2] = anchor2[2];
	anchormatrix[1][3] = receiveAnchor2;

	anchormatrix[2][0] = anchor3[0];
	anchormatrix[2][1] = anchor3[1];
	anchormatrix[2][2] = anchor3[2];
	anchormatrix[2][3] = receiveAnchor3;

	anchormatrix[3][0] = anchor4[0];
	anchormatrix[3][1] = anchor4[1];
	anchormatrix[3][2] = anchor4[2];
	anchormatrix[3][3] = receiveAnchor4;


	//Iteration steps currently 5
	for (int i=0;i<4;i++)
	{
		newton(anchormatrix,result);
	}
	//Ausgabe
	output[0] = result[0][0];
	output[1] = result[1][0];
	output[2] = result[2][0];
}

void modm::multilateration::newton(modm::Matrix<floatunit,4,4> anchormatrix,modm::Matrix<floatunit,4,1> &result)
{
	floatunit speedoflight = 299792548.f;
	//Calculate Jacobi Matrix and F = f(x,y,z,t) = spherefunction
	modm::Matrix<floatunit,4,4> jacobi;
	modm::Matrix<floatunit,4,1> f;
	for (int i = 0; i<4 ; i++){
		jacobi[i][0] = result[0][0] - 2*anchormatrix[i][0];
		jacobi[i][1] = result[1][0] - 2*anchormatrix[i][1];
		jacobi[i][2] = result[2][0] - 2*anchormatrix[i][2];
        jacobi[i][3] = (powl(speedoflight,2)*result[3][0]) - (2* powl(speedoflight,2)*anchormatrix[i][3]);
        f[i][0] = powl(anchormatrix[i][0]-result[0][0],2);
        f[i][0] += powl(anchormatrix[i][1]-result[1][0],2);
        f[i][0] += powl(anchormatrix[i][2]-result[2][0],2);
        f[i][0] -= powl(speedoflight,2) * powl(anchormatrix[i][3]-result[3][0],2);
		f[i][0] -= 2* f[i][0];
	}
	//solve LGS (Jacobi*z = -f) f(x,y,z,t) = spherefunction
	modm::LUDecomposition::solve(jacobi,&f);
	//Calculate (OLD)result - z
    for (int i= 0;i<4;i++){
        result[i][0] += f[i][0];
	}
	//done

}


void modm::multilateration::activemultilateration(Vector<floatunit, 3> &output,
												  Vector<floatunit, 3> anchor1,
												  Vector<floatunit, 3> anchor2,
												  Vector<floatunit, 3> anchor3,
												  Vector<floatunit, 3> anchor4,
												  floatunit receiveAnchor1,
												  floatunit receiveAnchor2,
												  floatunit receiveAnchor3,
												  floatunit receiveAnchor4)
{
	floatunit mini;
	mini = min(receiveAnchor1,receiveAnchor2);
	mini = min(mini,receiveAnchor3);
	mini = min(mini,receiveAnchor4);
	floatunit receiveTimeAnchor1 = receiveAnchor1-mini;
	floatunit receiveTimeAnchor2 = receiveAnchor2-mini;
	floatunit receiveTimeAnchor3 = receiveAnchor3-mini;
	floatunit receiveTimeAnchor4 = receiveAnchor4-mini;
	output = Vector<floatunit, 3>();
	floatunit speedoflight = 299792548.f;
	const floatunit A00A[9]  ={
		2*(anchor1.x-anchor4.x),
		2*(anchor1.y-anchor4.y),
		2*(anchor1.z-anchor4.z),
		2*(anchor2.x-anchor4.x),
		2*(anchor2.y-anchor4.y),
		2*(anchor2.z-anchor4.z),
		2*(anchor3.x-anchor4.x),
		2*(anchor3.y-anchor4.y),
		2*(anchor3.z-anchor4.z)
	};
    const floatunit A00ba[3] = {(powl(anchor1.x,2.0f)-powl(anchor4.x,2.0f))+(powl(anchor1.y,2.0f)-powl(anchor4.y,2.0f))+(powl(anchor1.z,2.0f)-powl(anchor4.z,2.0f))-((powl(speedoflight,2.0f)*powl(receiveTimeAnchor1,2.0f))-(powl(speedoflight,2.0f)*powl(receiveTimeAnchor4,2.0f))),
                                (powl(anchor2.x,2.0f)-powl(anchor4.x,2.0f))+(powl(anchor2.y,2.0f)-powl(anchor4.y,2.0f))+(powl(anchor2.z,2.0f)-powl(anchor4.z,2.0f))-((powl(speedoflight,2.0f)*powl(receiveTimeAnchor2,2.0f))-(powl(speedoflight,2.0f)*powl(receiveTimeAnchor4,2.0f))),
                                (powl(anchor3.x,2.0f)-powl(anchor4.x,2.0f))+(powl(anchor3.y,2.0f)-powl(anchor4.y,2.0f))+(powl(anchor3.z,2.0f)-powl(anchor4.z,2.0f))-((powl(speedoflight,2.0f)*powl(receiveTimeAnchor3,2.0f))-(powl(speedoflight,2.0f)*powl(receiveTimeAnchor4,2.0f)))
							   };
    const floatunit A0tba[3] = {	2*(powl(speedoflight,2)*(receiveTimeAnchor1-receiveTimeAnchor4)),
                                    2*(powl(speedoflight,2)*(receiveTimeAnchor2-receiveTimeAnchor4)),
                                    2*(powl(speedoflight,2)*(receiveTimeAnchor3-receiveTimeAnchor4))
							   };
	modm::Matrix<floatunit,3,1> A0tb(A0tba);
	modm::Matrix<floatunit,3,3> A00M(A00A);
	modm::Matrix<floatunit,3,1> A00b(A00ba);
	modm::LUDecomposition::solve(A00M,&A00b);
	modm::LUDecomposition::solve(A00M,&A0tb);


    floatunit A = powl((anchor4.x-A00b[0][0]),2.f)+powl((anchor4.y-A00b[1][0]),2.f)+powl((anchor4.z-A00b[2][0]),2.f) -powl((speedoflight*receiveTimeAnchor4),2.f);
    floatunit B = ((anchor4.x-A00b[0][0])*A0tb[0][0])+((anchor4.y-A00b[1][0])*A0tb[0][1])+((anchor4.z-A00b[2][0])*A0tb[2][0])-(powl(speedoflight,2)*receiveTimeAnchor4);
    floatunit C = powl(A0tb[0][0],2)+powl(A0tb[1][0],2)+powl(A0tb[2][0],2)-powl(speedoflight,2);

	floatunit t0 = (2 * B);
    floatunit t1 = powl(4*B,2.f)- (4*A*C);
	t0 = t0 + sqrt(abs(t1));
	t0 = t0 / (C != 0 ? 2*C : 1);

	output.x = A00b[0][0] + (A0tb[0][0]*t0);
	output.y = A00b[1][0] + (A0tb[1][0]*t0);
	output.z = A00b[2][0] + (A0tb[2][0]*t0);



}

void modm::multilateration::passivemultilateration(Vector<floatunit, 3> &output,
												   Vector<floatunit, 3> anchor1,
												   Vector<floatunit, 3> anchor2,
												   Vector<floatunit, 3> anchor3,
												   Vector<floatunit, 3> anchor4,
												   floatunit ReceiveTimeAnchor1,
												   floatunit ReceiveTimeAnchor2,
												   floatunit ReceiveTimeAnchor3,
												   floatunit ReceiveTimeAnchor4,
												   floatunit SendtimeAnchor1,
												   floatunit SendtimeAnchor2,
												   floatunit SendtimeAnchor3,
												   floatunit SendtimeAnchor4)
{
	floatunit time2 = (SendtimeAnchor2-SendtimeAnchor1);
	floatunit time3 = (SendtimeAnchor3-SendtimeAnchor1);
	floatunit time4 = (SendtimeAnchor4-SendtimeAnchor1);

    activemultilateration(output,anchor1,anchor2,anchor3,anchor4,
                          ReceiveTimeAnchor1,
                          (ReceiveTimeAnchor2 - time2),
                          (ReceiveTimeAnchor3 - time3),
                          (ReceiveTimeAnchor4 - time4));

}


void
modm::multilateration::trilateration(Vector<floatunit, 3> &output,
									 Vector<floatunit, 3> anchor0,
									 Vector<floatunit, 3> anchor1,
									 Vector<floatunit, 3> anchor2,
									 floatunit distanceToAnchor0,
									 floatunit distanceToAnchor1,
									 floatunit distanceToAnchor2)
{
	//Annahme Positionsarrays {x,y,z} in m und Distanz in m
	//Offset berechnen
	output.x = 0.0;
	output.y = 0.0;
	output.z = 0.0;
	Vector<floatunit,3> offset = Vector<floatunit,3>(anchor0.x,anchor0.y,anchor0.z);
	Vector<floatunit,3> newAnchor1 = Vector<floatunit,3>(anchor1.x-offset.x,anchor1.y-offset.y,anchor1.z-offset.z);
	Vector<floatunit,3> newAnchor2 = Vector<floatunit,3>(anchor2.x-offset.x,anchor2.y-offset.y,anchor2.z-offset.z);

	//Rotation berechnen -- Die von den drei Anchorpunkten aufgespannte Ebene wird in die XY Ebene rotiert
	float rotationangleXY = atan2 (newAnchor1.y,newAnchor1.x);
	modm::multilateration::rotate (2*M_PI-rotationangleXY,newAnchor1.x,newAnchor1.y);
	modm::multilateration::rotate (2*M_PI-rotationangleXY,newAnchor2.x,newAnchor2.y);
	floatunit rotationangleXZ = atan2 (newAnchor1.z,newAnchor1.x);
	modm::multilateration::rotate (2*M_PI-rotationangleXZ,newAnchor1.x,newAnchor1.z);
	modm::multilateration::rotate (2*M_PI-rotationangleXZ,newAnchor2.x,newAnchor2.z);
	floatunit rotationangleYZ = atan2 (newAnchor2.z,newAnchor2.y);
	modm::multilateration::rotate (2*M_PI-rotationangleYZ,newAnchor2.y,newAnchor2.z);

	//Berechne Output nach Pablo Cotera et al 2016 [Indoor Robot Positioning using an Enhanced Trilateration Algorithm]
    output.x = (powl(distanceToAnchor0,2)-powl(distanceToAnchor1,2)+powl(newAnchor1.x,2)) / (newAnchor1.x != 0 ? 2*newAnchor1.x : 1);
    output.y = ((powl(distanceToAnchor0,2)-powl(distanceToAnchor2,2))
                +powl(newAnchor2.x,2) + powl(newAnchor2.y,2)-(2*newAnchor2.x*output.x))
			/ (newAnchor2.y != 0 ? 2*newAnchor2.y : 1);

    floatunit zsqaured = powl(distanceToAnchor0,2.0) - powl(output.x,2.0) - powl(output.y,2.0);

	if (zsqaured < 0)
	{
		output.z = (-1) *  sqrt(abs(zsqaured));
	}
	else
	{
		output.z = sqrt(zsqaured);
	}

	//zurückrotieren
	modm::multilateration::rotate (rotationangleYZ,output.y,output.z);
	modm::multilateration::rotate (rotationangleXZ,output.x,output.z);
	modm::multilateration::rotate (rotationangleXY,output.x,output.y);
	//Offset aufrechnen
	output.x = output.x + offset.x;
	output.y = output.y + offset.y;
	output.z = output.z + offset.z;
	//done
}

void modm::multilateration::rotate(floatunit angle, floatunit &x, floatunit &y)
{
	floatunit xnew = floatunit(x * cos(angle) ) - floatunit(y * sin(angle));
	floatunit ynew = floatunit(y * cos(angle) ) + floatunit(x * sin(angle));
	x = xnew;
	y = ynew;
}

void modm::multilateration::trilaterationwithcorrection(Vector<floatunit, 3> &output,
									 Vector<floatunit, 3> anchor0,
									 Vector<floatunit, 3> anchor1,
									 Vector<floatunit, 3> anchor2,
									 Vector<floatunit, 3> anchor3,
									 floatunit distanceToAnchor0,
									 floatunit distanceToAnchor1,
									 floatunit distanceToAnchor2,
									 floatunit distanceToAnchor3)
{
	//Annahme Positionsarrays {x,y,z} in m und Distanz in m
	//Offset berechnen
	output.x = 0.0;
	output.y = 0.0;
	output.z = 0.0;
	Vector<floatunit,3> offset = Vector<floatunit,3>(anchor0.x,anchor0.y,anchor0.z);
	Vector<floatunit,3> newAnchor1 = Vector<floatunit,3>(anchor1.x-offset.x,anchor1.y-offset.y,anchor1.z-offset.z);
	Vector<floatunit,3> newAnchor2 = Vector<floatunit,3>(anchor2.x-offset.x,anchor2.y-offset.y,anchor2.z-offset.z);
	Vector<floatunit,3> newAnchor3 = Vector<floatunit,3>(anchor3.x-offset.x,anchor3.y-offset.y,anchor3.z-offset.z);

	//Rotation berechnen -- Die von den drei Anchorpunkten aufgespannte Ebene wird in die XY Ebene rotiert
	float rotationangleXY = atan2 (newAnchor1.y,newAnchor1.x);
	modm::multilateration::rotate (2*M_PI-rotationangleXY,newAnchor1.x,newAnchor1.y);
	modm::multilateration::rotate (2*M_PI-rotationangleXY,newAnchor2.x,newAnchor2.y);
	modm::multilateration::rotate (2*M_PI-rotationangleXY,newAnchor3.x,newAnchor3.y);
	floatunit rotationangleXZ = atan2 (newAnchor1.z,newAnchor1.x);
	modm::multilateration::rotate (2*M_PI-rotationangleXZ,newAnchor1.x,newAnchor1.z);
	modm::multilateration::rotate (2*M_PI-rotationangleXZ,newAnchor2.x,newAnchor2.z);
	modm::multilateration::rotate (2*M_PI-rotationangleXZ,newAnchor3.x,newAnchor3.z);
	floatunit rotationangleYZ = atan2 (newAnchor2.z,newAnchor2.y);
	modm::multilateration::rotate (2*M_PI-rotationangleYZ,newAnchor2.y,newAnchor2.z);
	modm::multilateration::rotate (2*M_PI-rotationangleYZ,newAnchor3.y,newAnchor3.z);

	//Berechne Output nach Pablo Cotera et al 2016 [Indoor Robot Positioning using an Enhanced Trilateration Algorithm]
    output.x = (powl(distanceToAnchor0,2)-powl(distanceToAnchor1,2)+powl(newAnchor1.x,2)) / (newAnchor1.x != 0 ? 2*newAnchor1.x : 1);
    output.y = ((powl(distanceToAnchor0,2)-powl(distanceToAnchor2,2))
                +powl(newAnchor2.x,2) + powl(newAnchor2.y,2)-(2*newAnchor2.x*output.x))
			/ (newAnchor2.y != 0 ? 2*newAnchor2.y : 1);

    floatunit zsqaured = powl(distanceToAnchor0,2.0) - powl(output.x,2.0) - powl(output.y,2.0);
    floatunit distancetopos = sqrt(powl(output.x-newAnchor3.x,2.0f)+powl(output.y-newAnchor3.y,2.0f)+powl(sqrt(abs(zsqaured))-newAnchor3.z,2.0f));
    floatunit distancetoneg = sqrt(powl(output.x-newAnchor3.x,2.0f)+powl(output.y-newAnchor3.y,2.0f)+powl((-1)*sqrt(abs(zsqaured))-newAnchor3.z,2.0f));

	if (abs(distancetopos-distanceToAnchor3)>abs(distancetoneg-distanceToAnchor3))
	{
		output.z = (-1) *  sqrt(abs(zsqaured));
	}
	else
	{
		output.z = sqrt(abs(zsqaured));
	}

	//zurückrotieren
	modm::multilateration::rotate (rotationangleYZ,output.y,output.z);
	modm::multilateration::rotate (rotationangleXZ,output.x,output.z);
	modm::multilateration::rotate (rotationangleXY,output.x,output.y);
	//Offset aufrechnen
	output.x = output.x + offset.x;
	output.y = output.y + offset.y;
	output.z = output.z + offset.z;
	//done
}




