"use client";

import { Box } from "@mui/material";
import mofelImg from "../res/mofel.png";

export default function Home() {
  

  return (
    <Box
      sx={{
        width: "100%",
        height: "100%",
        display: "flex",
        justifyContent: "center",
        alignItems: "center",
        bgcolor: "#fafafa",
        p: 2
      }}
    >
      <Box
        component="img"
        src={mofelImg.src}
        alt="MOFEL Product"
        sx={{
          maxWidth: "100%",
          maxHeight: "100%",
          objectFit: "contain",
          filter: "drop-shadow(0px 10px 20px rgba(0,0,0,0.15))"
        }}
      />
    </Box>
  );
}