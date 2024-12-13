import { BrowserRouter } from "react-router-dom";
import { Box, CssBaseline } from "@mui/material";
import Routing from "./routes/Routing";
import { Footer } from "./components";
import Sidenav from "./components/sidenav/sidenav";
import MainLayout from "./layout/main-layout";

const App = () => {
  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        height: "100vh",
        backgroundColor: "rgb(248, 249, 250)",
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
