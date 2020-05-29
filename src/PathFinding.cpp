#include <iostream>
#include "board.h"

int main()
{
    Board board;

    Point *start = new Point(8, 8);
    Point *goal = new Point(1, 1);
    board.find(*start, *goal);

    board.show();

    system("PAUSE");
}
