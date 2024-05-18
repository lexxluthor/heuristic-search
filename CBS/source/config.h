#ifndef CONFIG_H
#define	CONFIG_H
#include <string>
#include "tinyxml2.h"

class Config
{
    public:
        Config();
        ~Config();
        bool getConfig(const char *FileName);

        double*         SearchParams;
        std::string*    LogParams;
        unsigned int    N;
        int             searchType;
        int             lowLevel;
        int             minAgents = 1;
        int             maxAgents = -1;
        int             maxTime = 1000;
        std::string     agentsFile;
        int             tasksCount = 1;
        bool            withCAT = false;
        bool            withPerfectHeuristic = false;
        bool            singleExecution = false;
        bool            withCardinalConflicts = false;
        bool            withBypassing = false;
        bool            withMatchingHeuristic = false;
        bool            storeConflicts = false;
        bool            withDisjointSplitting = false;
        bool            withFocalSearch = false;
        bool            saveAggregatedResults = true;
        bool            useCatAtRoot = true;
        int             lowLevelRestartFrequency = 10000000;
        bool            withReplanning = false;
        double          focalW = 1.0;
        int             agentsStep = 1;
        int             firstTask = 1;

    private:
        static bool getValueFromText(tinyxml2::XMLElement *elem, const char *name, const char *typeName, void *field);
        static bool getValueFromAttribute(tinyxml2::XMLElement *elem, const char *elemName,
                                          const char *attrName, const char *typeName, void *field);
        static bool getText(tinyxml2::XMLElement *elem, const char *name, std::string &field);
        static tinyxml2::XMLElement* getChild(tinyxml2::XMLElement *elem, const char *name, bool printError = true);
};

#endif

