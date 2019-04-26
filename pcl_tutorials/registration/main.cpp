#include <iostream>
#include "registrationICP.h"
#include "registrationNDT.h"
#include "registrationSampleConsensus.h"
#include "template_alignment.h"
#include "cluster_recognition.h"
#include "registration_binlang.h"
//#include "segmentdifferences.h"

int main()
{
	//computeICP();

	//getInteractive_icp();

	//computeNDT();

	//computeSampleConsensusPrerejective();

	//template_alignment();

	/************************************************************************/
	/*                     vfh_recognition                                  */
	/*先运行build_tree，再运行find_nearest_neighbors							*/
	//build_tree();
	//find_nearest_neighbors();
	/************************************************************************/

	registration_binlang();

	getchar();
	return 0;
}