#include <commands_listener.hpp>

std::string state_maker(int i, int j)
{
    return std::to_string(i) + std::to_string(j);
}

/*
NAME: printQtableSA
PARAMETERS: aTable - map of action and Qvalue
PURPOSE: prints the action and Qvalues of a State
*/
void printQtableSA(std::map<int, float> aTable)
{
    for (auto it = aTable.begin();
         it != aTable.end(); ++it)
    {
        std::cout << "     Action" << it->first << " value" << it->second << std::endl;
    }
}

/*
NAME: printQtable
PARAMETERS: QTable - map of State and action table
PURPOSE: prints the entire Qtable
*/
void printQtable(std::map<std::string, std::map<int, float>> qtable)
{
    for (auto it = qtable.begin();
         it != qtable.end(); ++it)
    {
        std::cout << "State " << it->first << std::endl;
        printQtableSA(it->second);
    }
}

/*
NAME: largest_qval_in_map
PARAMETERS: sampleMap - map of int and float
PURPOSE: returns the largest value in the map
*/
std::pair<int, float> largest_qval_in_map(
    std::map<int, float> sampleMap)
{

    // Reference variable to help find
    // the entry with the highest value
    std::pair<int, float> entryWithMaxValue = std::make_pair(0, 0);

    // Iterate in the map to find the required entry
    std::map<int, float>::iterator currentEntry;
    for (currentEntry = sampleMap.begin();
         currentEntry != sampleMap.end();
         ++currentEntry)
    {

        // If this entry's value is more
        // than the max value
        // Set this entry as the max
        if (currentEntry->second > entryWithMaxValue.second)
        {

            entryWithMaxValue = std::make_pair(
                currentEntry->first,
                currentEntry->second);
        }
    }

    return entryWithMaxValue;
}
