import {
  ThemeProvider,
  createTheme,
  responsiveFontSizes,
} from "@mui/material/styles";
import React from "react";

type Props = {
  children?: React.ReactNode;
};

export const AppThemeProvider: React.FC<Props> = ({ children }) => {
  const theme = responsiveFontSizes(
    createTheme({
      typography: {
        fontFamily: "Open Sans, Helvetica, Arial, sans-serif",
      },
    }),
  );

  return <ThemeProvider theme={theme}>{children}</ThemeProvider>;
};