#ifndef AUTO_ITERATOR
#define AUTO_ITERATOR

#include <list>
#include <memory>
#include "auto/autoBase.h"

class iterator
{
private:
    std::list<std::unique_ptr<RobotAuto>> steps;
    std::unique_ptr<RobotAuto> currStep;
public:

    void addStep(std::unique_ptr<RobotAuto> NewStep)
    {
        steps.push_back(std::move(NewStep));
    }

    void Init()
    {
        if (!steps.empty()) { //if the list isnt empty 
            currStep = std::move(steps.front());
            steps.pop_front();
            currStep->AutoInit();
        }
    }

    int Run()
    {
        if (currStep && (currStep->AutoRun() > -1))  // Check if currStep is not null and if the step is done
        {
            if (!steps.empty()) 
            {
                currStep = std::move(steps.front());
                steps.pop_front();
                currStep->AutoInit();
            }
        }

        if(steps.empty())
        {
            return 0;
        }

        return -1;
    }

    void clear()
    {
        steps.clear();
    }
};

#endif