"use client"
import React, { useState, useEffect, useRef } from 'react';
import Slider from '@mui/material/Slider';
import Box from '@mui/material/Box';
import Grid from '@mui/material/Grid';
import { PanelContainer, MasconHandle, ReverserHandle, SemicircleMascon } from './style';

export default function TrainControllerApp() {
    const [masconValue, setMasconValue] = useState(0);
    const [reverserValue, setReverserValue] = useState(0);
    const [directionValue, setDirectionValue] = useState(0);

    const [scale, setScale] = useState(1);

    useEffect(() => {
        const handleResize = () => {
            const h = window.innerHeight;
            const threshold = 450;
            const newScale = h <= threshold ? h / threshold : 1;
            setScale(newScale);
        };

        handleResize();
        window.addEventListener('resize', handleResize);
        
        return () => window.removeEventListener('resize', handleResize);
    }, []);

    const handleMasconChange = (event: any, newValue: number | number[]) => {
        setMasconValue(Array.isArray(newValue) ? newValue[0] : newValue);
    };

    const handleReverserChange = (event: any, newValue: number | number[]) => {
        setReverserValue(Array.isArray(newValue) ? newValue[0] : newValue);
    };

    return (
        <Box sx={{ position: "absolute", top: 0, left: 0, width: "100vw", height: "100vh", backgroundColor: '#222222' }}>
            <Box sx={{ display: 'flex', flexDirection: 'column', width: "100%",
                            transform: `scale(${scale})`, 
                            transformOrigin: 'top center', 
                            transition: 'transform 0.1s linear' 
                        }}>
                <Box
                    sx={{
                        display: 'flex',
                        flexDirection: 'row',
                        alignItems: 'center',
                        justifyContent: 'center',
                    }}
                >
                    <Grid width="calc(100%/3)" display={"flex"} height={"100%"}>
                         {/* 1つ目のGrid: ここはそのまま */}
                        <PanelContainer sx={{ mr: 2, marginLeft: 5 }}>
                            <ReverserHandle
                                value={reverserValue}
                                onChange={handleReverserChange}
                                sx={{ height: 180 }}
                            />
                        </PanelContainer>
                        <PanelContainer sx={{ flexShrink: 3, height: "100%", scale: 1.0 }}>
                            <MasconHandle
                                value={masconValue}
                                onChange={handleMasconChange}
                                sx={{ marginTop: 2 }}
                            />
                        </PanelContainer>
                    </Grid>

                    {/* ★修正: 2つ目のGridに scale を適用 */}
                    <Grid 
                        width="calc(100%/3 - 10%)"
                    >
                        {/* 空白 ボタン等入れる予定 */}
                    </Grid>

                    <Grid width="calc(100%/3 + 10%)">
                        {/* 3つ目のGrid: ここはそのまま */}
                        <PanelContainer sx={{ p: 2 }}>
                            <SemicircleMascon
                                value={directionValue}
                                onChange={setDirectionValue}
                            />
                        </PanelContainer>
                    </Grid>
                </Box>
            </Box>
        </Box>
    );
}