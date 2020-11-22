/*********************************************************************
 *
 * This package is a support package for main monitor
 *
 * Author: Milan Michael
 *         
 *********************************************************************/
#include <sstream> // for string streams
#include <string>  // for string

namespace Utility
{
    /**
     * @brief This is used to convert data to string.
     * @param n data passed for the string convertion.
    */
    template <typename T>
    std::string ToString(const T &n)
    {
        std::ostringstream stm;
        stm << n;
        return stm.str();
    }

    /**
     * @brief This is used to convert data to string.
     * @param str The string/sentance need to be splited.
     * @param word_no The index value of the word required from the string/sentance.
    */
    std::string WordFromString(const std::string str, const int word_no)
    {
        std::string word = "";
        int string_no = 1;
        for (auto x : str)
        {
            if (x == ' ')
            {
                if (string_no == word_no)
                    return (word);
                string_no += 1;
                word = "";
            }
            else
                word = word + x;
        }
        return word;
    }

    /**
     * @brief This is used to print all vector elements for debugging.
     * @param a passed vector for printing.
    */
    template <typename T>
    void PrintVector(const T &a)
    {
        std::cout << "The vector elements are : \n";

        for (int i = 0; i < a.size(); i++)
            std::cout << a.at(i) << '\n';
    }
}; // namespace Utility
