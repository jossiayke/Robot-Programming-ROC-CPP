/**
 * @file main.cpp
 * @author Yoseph Kebede (ykebede2@terpmail.umd.edu)
 * @brief RWA1 Assignment Deliverable
 * @version 0.1
 * @date 2021-09-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

//*define your data structures here

struct s_Box{ //stores the information about small boxes
	unsigned int no_boxes{};
	unsigned int max_parts{};
};

struct m_Box{//stores the information about medium boxes
	unsigned int no_boxes{};
	unsigned int max_parts{};
};

struct l_Box{//stores the information about large boxes
	unsigned int no_boxes{};
	unsigned int max_parts{};
};

struct xl_Box{//stores the information about extra large boxes
	unsigned int no_boxes{};
	unsigned int max_parts{};
};

struct containers{
	s_Box sB{};
	m_Box mB{};
	l_Box lB{};
	xl_Box xB{};
};

/*function prototypes here*/

bool isNumber(const std::string& str);

bool isPos(int& input);

unsigned int get_total_parts();

void get_total_boxes(containers& boxes);

void get_part_per_box(containers&);

void fill_up_boxes(const containers&);

//*function definitions here*/

/**
 * @brief Read the user input and identify if type is string
 * 
 * @return boolean verifying entered number is int 
 */

bool isNumber(const std::string& str){

	return str.find_first_not_of("0123456789") == std::string::npos;
}

/**
 * @brief Read the user input and identify if type is positive integer
 * 
 * @return boolean verifying entered number is positive int
 */

bool isPos(const int& input){

	return (input == abs(input));

}
/**
 * @brief Get the total number of parts to be placed in box
 * 
 * @return unsigned Number of parts entered by the user 
 */

unsigned int get_total_parts(){

	std::string total_parts{};
	std::cout << "How many parts in total?: ";
	std::cin >> total_parts;

	while(!(isPos(std::stoi(total_parts)) & isNumber(total_parts))){
		std::cout << "You can only enter positive integers. So, how many parts in total?: ";
		std::cin >> total_parts;
	}

	return std::stoi(total_parts);
};

/**
 * @brief Get the total number of boxes for each type
 * 
 * @param struct to hold box  information
 */

void get_total_boxes(containers& boxes){
	std::string s, m, l, xl;

	std::cout << "How many boxes for S/M/L/XL? ";
	std::cin >> s >> m >> l >> xl;

	while (!(isNumber(s) & isNumber(m) & isNumber(l) & isNumber(xl))){
		std::cout << "You entered a non-integer value. \n";
		std::cout << "Enter again how many boxes exist for S/M/L/XL? ";
		std::cin >> s >> m >> l >> xl;
	}

	boxes.sB.no_boxes = std::stoi(s); 
	boxes.mB.no_boxes = std::stoi(m); 
	boxes.lB.no_boxes = std::stoi(l); 
	boxes.xB.no_boxes = std::stoi(xl);

	while (!(isPos(boxes.sB.no_boxes) & isPos(boxes.mB.no_boxes) & isPos(boxes.lB.no_boxes) & isPos(boxes.xB.no_boxes))){
		std::cout << "Can't enter a negative number. Please reenter input.\n";
		std::cout << "Again, how many boxes for S/M/L/XL? ";
		std::cin >> boxes.sB.no_boxes;
		std::cin>> boxes.mB.no_boxes;
		std::cin>> boxes.lB.no_boxes;
		std::cin>> boxes.xB.no_boxes;

	}


}

/**
* @brief Get the number of parts for each box type
* 
* @param <data_structure> Data structure to hold part information
*/
void get_part_per_box(containers& ppB){
	std::string s, m, l, xl;

	std::cout << "How many parts per box for S/M/L/XL? ";
	std::cin >> s >> m >> l >> xl;

	while (!(isNumber(s) & isNumber(m) & isNumber(l) & isNumber(xl))){
		std::cout << "You entered a non-integer value. \n";
		std::cout << "Enter again how many boxes exist for S/M/L/XL? ";
		std::cin >> s >> m >> l >> xl;
	}

	ppB.sB.no_boxes = std::stoi(s); 
	ppB.mB.no_boxes = std::stoi(m); 
	ppB.lB.no_boxes = std::stoi(l); 
	ppB.xB.no_boxes = std::stoi(xl);

	while (!(isPos(ppB.sB.no_boxes) & isPos(ppB.mB.no_boxes) & isPos(ppB.lB.no_boxes) & isPos(ppB.xB.no_boxes))){
		std::cout << "Can't enter a negative number. Please reenter input.\n";
		std::cout << "Again, how many boxes for S/M/L/XL? ";

		std::cin >> ppB.sB.no_boxes;
		std::cin>> ppB.mB.no_boxes;
		std::cin>> ppB.lB.no_boxes;
		std::cin>> ppB.xB.no_boxes;

	}

	while (((ppB.sB.max_parts < ppB.mB.max_parts) & (ppB.mB.max_parts < ppB.lB.max_parts) & (ppB.lB.max_parts < ppB.xB.max_parts)) != true){
		std::cout << "The Boxes should hold number of parts in incremental order. Please reenter input." << "\n";
		std::cout << "Again, how many parts per box for S/M/L/XL? ";
		std::cin >> ppB.sB.max_parts;
		std::cin>> ppB.mB.max_parts;
		std::cin>> ppB.lB.max_parts;
		std::cin>> ppB.xB.max_parts;
	
	}

}

/**
* @brief Get the number of parts for each box type
* 
* @param <data_structure> Data structure to hold part information
*/
void fill_up_boxes(const containers& fillBox, int total_parts){
	
	int remainder{};
	int xLBox = (total_parts / fillBox.xB.max_parts);
	remainder = total_parts % fillBox.xB.max_parts;

	std::cout<< "Boxes that can be built with " << total_parts   << " parts \n";
	std::cout<< "------------------------------------------------------- \n";
	std::cout<< "XL box ("<<fillBox.xB.no_boxes<<" x "<<fillBox.xB.max_parts<<" max): " << xLBox <<" -- remaining parts: "<<remainder<<"\n";

	int lBox = (remainder / fillBox.lB.max_parts);
	remainder = remainder % fillBox.lB.max_parts;
	std::cout<< "L box ("<<fillBox.lB.no_boxes<<" x "<<fillBox.lB.max_parts<<" max): " << lBox <<" -- remaining parts: "<<remainder<<"\n";

	int mBox = (remainder / fillBox.mB.max_parts);
	remainder = remainder % fillBox.mB.max_parts;
	std::cout<< "M box ("<<fillBox.mB.no_boxes<<" x "<<fillBox.mB.max_parts<<" max): " << mBox <<" -- remaining parts: "<<remainder<<"\n";

	int sBox = (remainder / fillBox.sB.max_parts);
	remainder = remainder % fillBox.sB.max_parts;
	std::cout<< "S box ("<<fillBox.sB.no_boxes<<" x "<<fillBox.sB.max_parts<<" max): " << sBox <<" -- remaining parts: "<<remainder<<"\n";

	std::cout<< "parts not in boxes: "<< remainder << "\n";

};

int main() {
	int total_parts{};	//stores the total number of available parts.

	// Initialize type of boxes
	s_Box	small{1, 2};
	m_Box 	medium{3 , 4};
	l_Box 	large{5, 6};
	xl_Box 	x_Large{7, 8};

	//Initialize the group of avaialble containers 
	containers boxes{small,medium,large,x_Large};

	//call function to get total number of parts
	total_parts = get_total_parts();

	//call function to get total number of boxes of each type
	get_total_boxes(boxes);

	//call function to get the max number of parts per box type
	get_part_per_box(boxes);

	//call function to fill up boxes and to display result
	fill_up_boxes(boxes, total_parts);

	std::cout<< "|-****************************************************-| \n";
	std::cout<< "||****************************************************|| \n";
	std::cout<< "||     *********     **      **        ******         || \n";
	std::cout<< "||		*********     * *     **        **    **       || \n";
	std::cout<< "||		**            ** *    **        **     **      || \n";		
	std::cout<< "||		**            **  *   **        **      **     || \n";
	std::cout<< "||		*********     **  *   **        **       **    || \n";
	std::cout<< "||		*********     **   *  **        **       **    || \n";
	std::cout<< "||		**            **   *  **        **      **     || \n";
	std::cout<< "||		**            **    * **        **     **      || \n";
	std::cout<< "||		*********     **    * **        **    **       || \n";
	std::cout<< "||		*********     **     ***        ******         || \n";
	std::cout<< "||****************************************************|| \n"; 
	std::cout<< "||****************************************************-| \n";
}

// ** Junk code **
/* while (((boxes.sB.no_boxes != abs(boxes.sB.no_boxes)) || (boxes.mB.no_boxes!= abs(boxes.mB.no_boxes)) || (boxes.lB.no_boxes!= abs(boxes.lB.no_boxes)) || (boxes.xB.no_boxes!= abs(boxes.xB.no_boxes)))){
		std::cout << "Can't enter a negative number. Please reenter input.\n";
		std::cout << "Again, how many boxes for S/M/L/XL? ";
		std::cin >> boxes.sB.no_boxes;
		std::cin>> boxes.mB.no_boxes;
		std::cin>> boxes.lB.no_boxes;
		std::cin>> boxes.xB.no_boxes;
	}

	while ((sizeof(boxes.sB.no_boxes) != sizeof(unsigned int)) || (sizeof(boxes.mB.no_boxes) != sizeof(unsigned int)) || (sizeof(boxes.lB.no_boxes) != sizeof(unsigned int)) || (sizeof(boxes.xB.no_boxes) != sizeof(unsigned int))){
		std::cout << "Wrong input type entered. Please reenter input." << "\n";
		std::cout << "Again, how many boxes for S/M/L/XL? ";
		std::cin >> boxes.sB.no_boxes;
		std::cin>> boxes.mB.no_boxes;
		std::cin>> boxes.lB.no_boxes;
		std::cin>> boxes.xB.no_boxes;
	} */