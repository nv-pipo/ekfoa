#include "print.hpp"

void print_e16(Eigen::MatrixXd var){
	FILE * pFile;
	pFile = fopen ("var.txt","w");

	for (int i=0 ; i<var.rows() ; i++){
		for (int j=0 ; j<var.cols() ; j++){
			if (var(i, j)<0)
				fprintf (pFile, "  ");
			else
				fprintf (pFile, "   ");

			fprintf (pFile, "%1.16e", var(i, j));
		}
		fprintf (pFile, "\n");
	}

	fclose (pFile);
	std::cout << "pausing..." << std::endl;
	std::cin.ignore(1);
}

void print_d(Eigen::MatrixXd var){
	FILE * pFile;
	pFile = fopen ("var.txt","w");

	for (int i=0 ; i<var.rows() ; i++){
		for (int j=0 ; j<var.cols() ; j++){
			fprintf (pFile, "%d", (int)var(i, j));
			fprintf (pFile, "\t");
		}
		fprintf (pFile, "\n");
	}

	fclose (pFile);
	std::cout << "pausing..." << std::endl;
	std::cin.ignore(1);
}
