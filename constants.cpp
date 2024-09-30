// using enum class constants in c++: items::FridgeItems::RED_CAN

namespace items{
    enum class FridgeItems{
        RED_CAN,
        BLUE_CAN,
        GREEN_CAN,
        CLEAR_BOTTLE,
        BREAD,
        FRUIT
    };

    enum class TrashItems{
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
    enum class GlobalPositions{
        STARTING
    };

    enum class FridgeGraspPositions{
        LEFT,
        MIDDLE,
        RIGHT
    };

    enum class FridgeManeuverPositions{
        OPEN_DOOR,
        SCANNING
    };

    enum class TrashManeuverPositions{
        SCANNING,
        THROW_TRASH
    };
}

namespace arm_pos{
    enum class GlobalPositons{
        RESTING,
    };

    enum class ScanningPositions{
        FRIDGE,
        GROUND
    };
}

