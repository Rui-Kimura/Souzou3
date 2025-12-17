"use client";

import { useEffect, useState, useRef } from "react";
import { Box, Typography } from "@mui/material";
import Map from '@/components/Map';

export default function Page() {
  const containerRef = useRef<HTMLDivElement>(null);
  const contentRef = useRef<HTMLDivElement>(null);
  const [scale, setScale] = useState(1);

  useEffect(() => {
    const handleResize = () => {
      if (containerRef.current && contentRef.current) {
        const container = containerRef.current.getBoundingClientRect();
        
        const contentW = contentRef.current.scrollWidth;
        const contentH = contentRef.current.scrollHeight;

        if (contentW === 0 || contentH === 0) return;

        const scaleW = container.width / contentW;
        const scaleH = container.height / contentH;
        const newScale = Math.min(scaleW, scaleH);

        setScale(newScale * 0.95);
      }
    };

    handleResize();
    window.addEventListener("resize", handleResize);

    const observer = new ResizeObserver(handleResize);
    if (contentRef.current) {
      observer.observe(contentRef.current);
    }

    return () => {
      window.removeEventListener("resize", handleResize);
      observer.disconnect();
    };
  }, []);

  return (
    <Box
      sx={{
        position: "fixed",
        top: 0,
        left: 0,
        width: "100vw",
        height: "100vh",
        overflow: "hidden",
        bgcolor: "background.default",
        py: 4,
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        boxSizing: "border-box",
      }}
    >
      <Typography variant="h5" component="h1" sx={{ mb: 3, color: "text.primary", fontWeight: 'bold', flexShrink: 0 }}>
        地図
      </Typography>
      <Box
        ref={containerRef}
        sx={{
          flex: 1,
          width: "100%",
          display: "flex",
          justifyContent: "center",
          alignItems: "center",
          overflow: "hidden",
          position: "relative",
        }}
      >
        <Box
          ref={contentRef}
          sx={{
            display: "inline-block",
            transform: `scale(${scale})`,
            transformOrigin: "center center",
            transition: "transform 0.1s ease-out",
          }}
        >
          <Map />
        </Box>
      </Box>
    </Box>
  );
}