#include "board.h"

void Board::show() const 
{
	std::cout << std::endl;

	for (int y = 0; y < BOARD_SIZE; y++) {
		std::cout << " ";
		for (int x = 0; x < BOARD_SIZE; x++) {
			std::cout << "+-";
		}
		std::cout << "+" << std::endl;

		std::cout << " ";
		for (int x = 0; x < BOARD_SIZE; x++) {
			std::cout << "|";
			switch (mass_[y][x].getStatus()) {
			case Mass::status::BLANK:
				std::cout << " ";
				break;
			case Mass::status::START:
				std::cout << "S";
				break;
			case Mass::status::GOAL:
				std::cout << "G";
				break;
			case Mass::status::WAYPOINT:
				std::cout << "o";
				break;
			case Mass::status::WALL:
				std::cout << "#";
				break;
			case Mass::status::WATER:
				std::cout << "~";
				break;
			case Mass::status::ROAD:
				std::cout << "$";
				break;
			}
		}
		std::cout << "|" << std::endl;
	}

	std::cout << " ";
	for (int x = 0; x < BOARD_SIZE; x++) {
		std::cout << "+-";
	}
	std::cout << "+" << std::endl;

}
