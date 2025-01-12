import {
  CONTROLLER_CONTAINER_SIZE,
  DIRECTION_TYPES,
  IJoystickUpdateEvent,
} from "@/types";
import {
  KeyboardArrowUp,
  KeyboardArrowRight,
  KeyboardArrowDown,
  KeyboardArrowLeft,
} from "@mui/icons-material";
import { Box } from "@mui/material";
import { ReactNode } from "react";

interface JoystickWrapperProps {
  children: ReactNode;
  joystickStatus?: IJoystickUpdateEvent;
}

const getArrowColor = (currentDirection: any, direction: DIRECTION_TYPES) =>
  currentDirection !== direction ? "#ddd" : "#50C878";

export const JoystickWrapper = ({
  children,
  joystickStatus,
}: JoystickWrapperProps) => {
  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        width: CONTROLLER_CONTAINER_SIZE,
        height: CONTROLLER_CONTAINER_SIZE,
        position: "relative",
      }}
    >
      <Box
        sx={{
          position: "absolute",
          zIndex: 2,
          width: 1,
          height: 1,
          top: 0,
          left: 0,
        }}
      >
        <Box
          sx={{
            position: "absolute",
            display: "inline-flex",
            top: "-2px",
            left: "50%",
            transform: "translate(-50%,50%)",
          }}
        >
          <KeyboardArrowUp
            sx={{
              transform: "scale(1.6)",
              color: getArrowColor(
                joystickStatus?.direction,
                DIRECTION_TYPES.FORWARD,
              ),
            }}
          />
        </Box>
        <Box
          sx={{
            position: "absolute",
            display: "inline-flex",
            right: "-2px",
            top: "50%",
            transform: "translate(-50%, -50%)",
          }}
        >
          <KeyboardArrowRight
            sx={{
              transform: "scale(1.6)",
              color: getArrowColor(
                joystickStatus?.direction,
                DIRECTION_TYPES.RIGHT,
              ),
            }}
          />
        </Box>
        <Box
          sx={{
            position: "absolute",
            display: "inline-flex",
            bottom: "-2px",
            left: "50%",
            transform: "translate(-50%, -50%)",
          }}
        >
          <KeyboardArrowDown
            sx={{
              transform: "scale(1.6)",
              color: getArrowColor(
                joystickStatus?.direction,
                DIRECTION_TYPES.BACKWARD,
              ),
            }}
          />
        </Box>
        <Box
          sx={{
            position: "absolute",
            display: "inline-flex",
            left: "-2px",
            top: "50%",
            transform: "translate(50%, -50%)",
          }}
        >
          <KeyboardArrowLeft
            sx={{
              transform: "scale(1.6)",
              color: getArrowColor(
                joystickStatus?.direction,
                DIRECTION_TYPES.LEFT,
              ),
            }}
          />
        </Box>
      </Box>
      <Box
        sx={{
          "& > div": {
            boxShadow: "0 0 5px #d5dadb inset",
            border: "1px solid #fff",
            borderRadius: "50%",
            "& button": {
              zIndex: 5,
              background:
                "linear-gradient(to bottom,#ffffff,#eaf4fe) !important",
              boxShadow: "0 0 16px #d5dadb",
              border: "1px solid #d9dde4 !important",
            },
          },
        }}
      >
        {children}
      </Box>
    </Box>
  );
};
