import { BrowserRouter } from "react-router-dom";
import { Box, CssBaseline } from "@mui/material";
import { Routing } from "./routes";
import { Footer, Sidenav } from "./components";
import { MainLayout } from "./layout";

const App = () => {
  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        height: "100%",
        backgroundColor: "#f8f9fa",
      }}
    >
      <BrowserRouter>
        <CssBaseline />
        <MainLayout>
          <Sidenav>
            <Routing />
          </Sidenav>
        </MainLayout>
        <Footer />
      </BrowserRouter>
    </Box>
  );
};

export default App;
