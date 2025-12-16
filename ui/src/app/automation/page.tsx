"use client"
import React, { useState, useEffect, useRef } from 'react';
import { Box, Paper, Typography } from "@mui/material";
import Map from '@/components/Map'


interface Point {
    name: string,
    x: number;
    y: number;
    angle: number;
}

export default function Page() {
    const [points, setPoints] = useState<Point[]>();

    return(
    <Box sx={{ display: "flex", flexDirection: "column", alignItems: "center" }}>
      <Paper sx={{ width: "80%", p: 2 }}>
        <Box>
            <Typography>自動移動</Typography>
            <Map/>
            </Box>
        </Paper>
    </Box>

    )
}
