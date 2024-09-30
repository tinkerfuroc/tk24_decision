#include <string>


namespace items{
    enum FridgeItems{
        RED_CAN,
        BLUE_CAN,
        GREEN_CAN,
        CLEAR_BOTTLE,
        BREAD,
        FRUIT
    };

    enum TrashItems{
        CLEAR_BOTTLE,
        PLASTIC_BAG,
        RED_PAPER_BALL,
        WHITE_PAPER_BALL,
        BLUE_PAPER_BALL,
        YELLOW_PAPER_BALL,
        BLUE_CAN,
        RED_CAN,
    };
}

namespace tinker_pos{
    enum GlobalPositions{
        STARTING
    };

    enum FridgeGraspPositions{
        LEFT,
        MIDDLE,
        RIGHT
    };

    enum FridgeManeuverPositions{
        OPEN_DOOR,
        SCANNING
    };

    enum TrashManeuverPositions{
        SCANNING,
        THROW_TRASH
    };
}

namespace arm_pos{
    enum GlobalPositons{
        RESTING,
    };

    enum ScanningPositions{
        FRIDGE,
        GROUND
    };
}

