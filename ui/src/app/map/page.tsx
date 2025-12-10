"use client";

import { useEffect, useState } from "react";
import {
  Card,
  Box,
  Typography,
  Container,
  Stack,
  Paper
} from "@mui/material";

import Map from '@/components/Map'

export default function Page() {
  return (
    <Box
      sx={{
        minHeight: "100vh",
        bgcolor: "#f5f6fa",
        py: 4,
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
      }}
    >
        <Typography variant="h5" component="h1" sx={{ mb: 3, color: "#333", fontWeight: 'bold' }}>
          地図
        </Typography>
        <Map/>
    </Box>
  );
}