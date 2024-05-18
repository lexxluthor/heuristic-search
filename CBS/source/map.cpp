#include "map.h"


Map::Map()
{
    height = -1;
    width = -1;
    Grid = nullptr;
    cellSize = 1;
}

Map::~Map()
{
    if (Grid) {
        for (int i = 0; i < height; ++i)
            delete[] Grid[i];
        delete[] Grid;
    }
}

bool Map::CellIsTraversable(int i, int j, const std::unordered_set<Node> &occupiedNodes) const
{
    return (Grid[i][j] == CN_GC_NOOBS) && occupiedNodes.find(Node(i, j)) == occupiedNodes.end();
}

bool Map::CellIsObstacle(int i, int j) const
{
    return (Grid[i][j] != CN_GC_NOOBS);
}

bool Map::CellOnGrid(int i, int j) const
{
    return (i < height && i >= 0 && j < width && j >= 0);
}

bool Map::getMap(const char *FileName)
{
    int rowiter = 0, grid_i = 0, grid_j = 0;
    emptyCellCount = 0;

    tinyxml2::XMLElement *root, *map, *element, *mapnode;

    std::string value;
    std::stringstream stream;

    bool hasGridMem = false, hasGrid = false, hasHeight = false, hasWidth = false;

    tinyxml2::XMLDocument doc;

    // Load XML File
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS) {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }
    // Get ROOT element
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root) {
        std::cout << "Error! No '" << CNS_TAG_ROOT << "' tag found in XML file!" << std::endl;
        return false;
    }

    // Get MAP element
    map = root->FirstChildElement(CNS_TAG_MAP);
    if (!map) {
        std::cout << "Error! No '" << CNS_TAG_MAP << "' tag found in XML file!" << std::endl;
        return false;
    }

    for (mapnode = map->FirstChildElement(); mapnode; mapnode = mapnode->NextSiblingElement()) {
        element = mapnode->ToElement();
        value = mapnode->Value();
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);

        if (value == CNS_TAG_GRID) {
            hasGrid = true;

            element->QueryIntAttribute(CNS_TAG_HEIGHT, &height);
            if (height < 0) {
                std::cout << "Warning! Invalid value of '" << CNS_TAG_HEIGHT
                          << "' attribute encountered (or could not convert to integer)." << std::endl;
                std::cout << "Value of '" << CNS_TAG_HEIGHT << "' attribute should be an integer >=0" << std::endl;
            } else {
                hasHeight = true;
            }

            element->QueryIntAttribute(CNS_TAG_WIDTH, &width);
            if (width < 0) {
                std::cout << "Warning! Invalid value of '" << CNS_TAG_WIDTH
                          << "' attribute encountered (or could not convert to integer)." << std::endl;
                std::cout << "Value of '" << CNS_TAG_WIDTH << "' attribute should be an integer AND >0" << std::endl;
            } else {
                hasWidth = true;
            }

            if (!(hasHeight && hasWidth)) {
                std::cout << "Error! No '" << CNS_TAG_WIDTH << "' attribute or '" << CNS_TAG_HEIGHT << "' attribute in '"
                          << CNS_TAG_GRID << "' tag encountered!" << std::endl;
                return false;
            }

            Grid = new int *[height];
            for (int i = 0; i < height; ++i)
                Grid[i] = new int[width];

            element = mapnode->FirstChildElement();
            while (grid_i < height) {
                if (!element) {
                    std::cout << "Error! Not enough '" << CNS_TAG_ROW << "' tags inside '" << CNS_TAG_GRID << "' tag."
                              << std::endl;
                    std::cout << "Number of '" << CNS_TAG_ROW
                              << "' tags should be equal (or greater) than the value of '" << CNS_TAG_HEIGHT
                              << "' tag which is " << height << std::endl;
                    return false;
                }
                std::string str = element->GetText();
                std::vector<std::string> elems;
                std::stringstream ss(str);
                std::string item;
                while (std::getline(ss, item, ' '))
                    elems.push_back(item);
                rowiter = grid_j = 0;
                int val;
                if (!elems.empty())
                    for (grid_j = 0; grid_j < width; ++grid_j) {
                        if (grid_j == elems.size())
                            break;
                        stream.str("");
                        stream.clear();
                        stream << elems[grid_j];
                        stream >> val;
                        Grid[grid_i][grid_j] = val;
                        if (val == CN_GC_NOOBS) {
                            ++emptyCellCount;
                        }
                    }

                if (grid_j != width) {
                    std::cout << "Invalid value on " << CNS_TAG_GRID << " in the " << grid_i + 1 << " " << CNS_TAG_ROW
                              << std::endl;
                    return false;
                }
                ++grid_i;

                element = element->NextSiblingElement();
            }
        }
    }
    //some additional checks
    if (!hasGrid) {
        std::cout << "Error! There is no tag 'grid' in xml-file!\n";
        return false;
    }

    return true;
}

int Map::getValue(int i, int j) const
{
    if (i < 0 || i >= height)
        return -1;

    if (j < 0 || j >= width)
        return -1;

    return Grid[i][j];
}

int Map::getMapHeight() const
{
      return height;
}

int Map::getMapWidth() const
{
      return width;
}

int Map::getEmptyCellCount() const {
    return emptyCellCount;
}

double Map::getCellSize() const
{
      return cellSize;
}

int Map::getCellDegree(int i, int j) const {
    int degree = 0;
    for (int di = -1; di <= 1; ++di) {
        for (int dj = -1; dj <= 1; ++dj) {
            if ((di == 0) ^ (dj == 0)) {
                if (CellOnGrid(i + di, j + dj) && !CellIsObstacle(i + di, j + dj)) {
                    ++degree;
                }
            }
        }
    }
    return degree;
}





