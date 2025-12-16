"use client"
import React, { useState, useEffect, useRef } from 'react';
import { Box, Typography, Card } from "@mui/material";
import Map from '@/components/Map'

interface Point {
    name: string,
    x: number;
    y: number;
    angle: number;
}

export default function Page() {
    const [points, setPoints] = useState<Point[]>([]);
    
    const containerRef = useRef<HTMLDivElement>(null);
    const contentRef = useRef<HTMLDivElement>(null);
    const [scale, setScale] = useState(1);

    useEffect(() => {
        const fetchPoints = async () => {
            try {
                const response = await fetch('http://localhost:8100/saved_target_points');
                const data = await response.json();
                setPoints(data);
            } catch (error) {
                console.error('Error fetching points:', error);
            }
        }
        fetchPoints();
    }, []);

    useEffect(() => {
        const container = containerRef.current;
        const content = contentRef.current;
        if (!container || !content) return;

        const updateScale = () => {
            const containerRect = container.getBoundingClientRect();
            
            // コンテンツ(Map)の本来のサイズを取得
            const contentWidth = content.offsetWidth; 
            const contentHeight = content.offsetHeight;
            const containerWidth = containerRect.width;
            const containerHeight = containerRect.height;

            if (contentWidth === 0 || contentHeight === 0) return;

            const scaleX = containerWidth / contentWidth;
            const scaleY = containerHeight / contentHeight;
            
            // 縦横小さい方に合わせてフィットさせる
            const newScale = Math.min(scaleX, scaleY);
            
            // 縮小のみ（拡大したい場合は : 1 を削除）
            setScale(newScale < 1 ? newScale : 1);
        };

        const observer = new ResizeObserver(() => {
            updateScale();
        });

        observer.observe(container);
        observer.observe(content);
        updateScale(); // 初期実行

        return () => observer.disconnect();
    }, []);

    return (
        <Box sx={{ 
            // 【変更点】position: fixed で画面に完全固定し、スクロールを排除
            position: "fixed",
            inset: 0, // top: 0, right: 0, bottom: 0, left: 0 と同じ
            width: "100%",
            height: "100%",
            overflow: "hidden", // 全体のスクロールバーを禁止
            
            display: "flex", 
            flexDirection: "column", 
            bgcolor: "background.default",
            p: 2, // 全体のパディング
            boxSizing: "border-box" // パディングを含めてサイズ計算
        }}>
            <Box sx={{ pb: 2, flexShrink: 0 }}>
                <Card sx={{ p: 1 }}>
                    <Typography>自動移動</Typography>
                </Card>
            </Box>

            <Box sx={{ 
                display: "flex", 
                flex: 1, 
                overflow: "hidden", 
                flexDirection: { xs: "column", md: "row" }, 
                gap: 2 
            }}>
                
                {/* マップ表示エリア */}
                <Box 
                    ref={containerRef}
                    sx={{ 
                        flex: 1, 
                        display: "flex", 
                        justifyContent: "center", 
                        alignItems: "center",
                        overflow: "hidden", 
                        border: "1px solid #ddd", 
                        borderRadius: 1,
                        bgcolor: "#fff",
                        position: "relative"
                    }}
                >
                    <Box 
                        ref={contentRef}
                        sx={{
                            transform: `scale(${scale})`,
                            transformOrigin: "center center",
                            width: "max-content", 
                            height: "max-content",
                            display: "flex"
                        }}
                    >
                        <Map />
                    </Box>
                </Box>

                {/* 地点一覧エリア */}
                <Box sx={{ 
                    width: { xs: "100%", md: "300px" }, 
                    height: "100%", 
                    display: "flex",
                    flexDirection: "column",
                    overflow: "hidden"
                }}>
                    <Card sx={{ p: 1, mb: 1, flexShrink: 0 }}>
                        <Typography fontWeight="bold">地点一覧</Typography>
                    </Card>
                    
                    <Box sx={{ 
                        flex: 1, 
                        overflowY: "auto", 
                        pr: 1 
                    }}>
                        {
                            points.map((point, index) => (
                                <Box key={index} sx={{ border: "1px solid #ccc", p: 1, mb: 1, borderRadius: 1, bgcolor: "background.paper" }}>
                                    <Typography variant="subtitle2" fontWeight="bold">{point.name}</Typography>
                                    <Box sx={{ pl: 1, fontSize: "0.875rem" }}>
                                        <Typography variant="caption" display="block">X: {point.x}</Typography>
                                        <Typography variant="caption" display="block">Y: {point.y}</Typography>
                                        <Typography variant="caption" display="block">角度: {point.angle}</Typography>
                                    </Box>
                                </Box>
                            ))
                        }
                    </Box>
                </Box>
            </Box>
        </Box>
    )
}