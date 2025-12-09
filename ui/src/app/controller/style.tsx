"use client"
import { styled } from '@mui/material/styles';
import Slider from '@mui/material/Slider';
import Box from '@mui/material/Box';
import Paper from '@mui/material/Paper';
import { createTheme, ThemeProvider } from '@mui/material/styles';
import CssBaseline from '@mui/material/CssBaseline';
import Switch from '@mui/material/Switch';
import type { ReactNode } from 'react';
import { useEffect, useRef, useState } from 'react';

export const PanelContainer = styled(Paper)({
  padding: '24px',
  display: 'flex',
  flexDirection: 'column',
  alignItems: 'center',
  justifyContent: 'center',
  borderRadius: '4px',
  backgroundColor: '#000000ff',
  border: '3px solid #303030ff',
  boxShadow: '0 2px 4px rgba(0,0,0,0.05)',
  height: '100%',
});


const MasconSliderStyle = styled(Slider)({
  color: '#455a64',
  height: '100%',
  minHeight: 300,
  padding: '0 !important',

  '& .MuiSlider-track': {
    border: 'none',
    width: 12,
    backgroundColor: '#455a64',
  },
  '& .MuiSlider-rail': {
    opacity: 0.2,
    width: 12,
    backgroundColor: '#b1b1b1ff',
  },
  '& .MuiSlider-thumb': {
    height: 50,
    width: 100,
    backgroundColor: '#242424ff',
    border: '3px solid #c7c7c7ff',
    borderRadius: 45,
    boxShadow: '0 2px 4px rgba(0,0,0,0.2)',
    '&:focus, &:hover, &.Mui-active': {
      boxShadow: '0 4px 8px rgba(0,0,0,0.2)',
    },
    '&:before': { display: 'none' },
  },
  '& .MuiSlider-mark': {
    backgroundColor: '#ffffffd4',
    width: 50,
    left: '35px',
  },
  '& .MuiSlider-markLabel': {
    color: '#ffffffff',
    fontSize: '0.875rem',
    fontWeight: '500',
    left: '70px',
  },
  // EB
  '& .MuiSlider-mark[data-index="0"]': { backgroundColor: '#ff0000ff', height: 40, top: "-20px" },
  '& .MuiSlider-markLabel[data-index="0"]': { color: '#ff0000ff', fontWeight: 'bold', fontSize: '1rem' },

  // N
  '& .MuiSlider-mark[data-index="1"]': { backgroundColor: '#ffd500ff', height: "20px", top: "47px" },
  '& .MuiSlider-markLabel[data-index="1"]': { color: '#ffffffff' },
  // P1
  '& .MuiSlider-mark[data-index="2"]': { height: "20px", top: "105px" },
  // P2
  '& .MuiSlider-mark[data-index="3"]': { height: "25px", top: "163px" },

  // P3
  '& .MuiSlider-mark[data-index="4"]': { height: "35px", top: "215px" },
  // P4
  '& .MuiSlider-mark[data-index="5"]': { height: "40px", top: "270px" },
  // EB
  '& .MuiSlider-mark[data-index="6"]': { height: "45px", top: "325px" },
});

const MASCON_MARKS = [
  { value: 1, label: 'EB' },
  { value: 0, label: 'N' },
  { value: -1, label: 'P1' },
  { value: -2, label: 'P2' },
  { value: -3, label: 'P3' },
  { value: -4, label: 'P4' },
  { value: -5, label: 'P5' },
];

/**
 * MasconHandle Component
 * @param {number} value 
 * @param {function} onChange
 * @param {object} sx
 * @param {ReactNode} children
 * @param {object} other 
 */

type MasconHandleProps = {
  value: number;
  onChange: (event: Event, value: number | number[]) => void;
  sx?: object;
  children?: ReactNode;
  [key: string]: any;
};

export const MasconHandle = ({ value, onChange, sx, children, ...other }: MasconHandleProps) => {
  return (
    <Box
      sx={{
        height: "350px",
        display: 'flex',
        justifyContent: 'center',
        paddingLeft: 5,
        paddingRight: 10,
        paddingBottom: 3,
        position: 'relative',
        ...sx
      }}
    >
      <MasconSliderStyle
        orientation="vertical"
        value={value}
        min={-5}
        max={1}
        step={1}
        marks={MASCON_MARKS}
        onChange={onChange}
        valueLabelDisplay="off"
        track={false}
        aria-label="Master Controller Handle"
        {...other}
      />
      {children}
    </Box>
  );
};


const ReverserSliderStyle = styled(Slider)({
  height: '100%',
  minHeight: 120,
  padding: '0 !important',

  '& .MuiSlider-track': {
    border: 'none',
    width: 10,
    backgroundColor: '#484848ff',
  },
  '& .MuiSlider-rail': {
    width: 10,
    backgroundColor: '#fdfdfdff',
  },
  '& .MuiSlider-thumb': {
    height: 50,
    width: 50,
    backgroundColor: '#000000ff',
    border: '3px solid #5b5b5bff',
    borderRadius: 90,
    boxShadow: '0 2px 4px rgba(0,0,0,0.2)',
    '&:focus, &:hover, &.Mui-active': {
      boxShadow: '0 4px 8px rgba(0,0,0,0.2)',
    },
    '&:before': { display: 'none' },
  },
  '& .MuiSlider-mark': {
    backgroundColor: '#ffffffcc',
    height: 20,
    width: 40,
    left: '30px',
  },
  '& .MuiSlider-markLabel': {
    color: '#ffffffff',
    fontSize: '0.875rem',
    left: '55px',
  },
  // N
  '& .MuiSlider-mark[data-index="0"]': { top: "-10px" },
  '& .MuiSlider-mark[data-index="1"]': { top: "80px", backgroundColor: '#70CF6E' },
  '& .MuiSlider-mark[data-index="2"]': { top: "168px" },
});

const REVERSER_MARKS = [
  { value: 1, label: '前' },
  { value: 0, label: '切' },
  { value: -1, label: '後' },
];

/**
 * ReverserHandle Component
 * @param {number} value - 現在の位置 (1:前, 0:切, -1:後)
 * @param {function} onChange - 値変更時のコールバック
 * @param {object} sx - ルートBoxへの追加スタイル
 * @param {ReactNode} children - 追加の子要素
 * @param {object} other - Sliderコンポーネントへ渡すその他のprops
 */

type ReverserHandleProps = {
  value: number;
  onChange: (event: Event, value: number | number[]) => void;
  sx?: object;
  children?: ReactNode;
  [key: string]: any;
};

export const ReverserHandle = ({ value, onChange, sx, children, ...other }: ReverserHandleProps) => {
  return (
    <Box
      sx={{
        height: 200,
        display: 'flex',
        justifyContent: 'center',
        position: 'relative',
        ...sx
      }}
    >
      <ReverserSliderStyle
        orientation="vertical"
        value={value}
        min={-1}
        max={1}
        step={1}
        marks={REVERSER_MARKS}
        onChange={onChange}
        valueLabelDisplay="off"
        aria-label="Reverser Handle"
        {...other}
      />
      {children}
    </Box>
  );
};

const HANDLE_RADIUS = 130;
const CENTER_X = 180;
const CENTER_Y = 60;
const STROKE_WIDTH = 6;

// 角度変換
const valueToAngle = (value: any) => {
  const percent = (value + 100) / 200;
  return Math.PI * (1 - percent);
};

const angleToValue = (radians: any) => {
  let angle = radians;
  if (angle < 0) angle = 0;
  if (angle > Math.PI) angle = Math.PI;

  const percent = 1 - (angle / Math.PI);
  return (percent * 200) - 100;
};

/**
 * SemicircleMascon Component
 */
export const SemicircleMascon = ({ value, onChange, sx }: { value: number; onChange: (value: number) => void; sx?: object }) => {
  const svgRef = useRef<SVGSVGElement | null>(null);
  const [isDragging, setIsDragging] = useState(false);
  const [isMounted, setIsMounted] = useState(false);

  useEffect(() => {
    setIsMounted(true);
  }, []);

  const handleInteraction = (clientX: number, clientY: number): void => {
    if (!svgRef.current) return;

    if (typeof svgRef.current.getBoundingClientRect !== 'function') return;

    const rect = svgRef.current.getBoundingClientRect();
    const x = clientX - rect.left - CENTER_X;
    const y = clientY - rect.top - CENTER_Y;

    let angle: number = Math.atan2(y, x);

    if (angle < 0) {
      if (angle < -Math.PI / 2) angle = Math.PI;
      else angle = 0;
    }

    let newValue: number = angleToValue(angle);

    const step: number = 12.5;
    newValue = Math.round(newValue / step) * step;

    if (newValue < -100) newValue = -100;
    if (newValue > 100) newValue = 100;

    if (newValue === 0) newValue = 0;

    onChange(newValue);
  };

  const onMouseDown = (e: any) => {
    setIsDragging(true);
    handleInteraction(e.clientX, e.clientY);
  };

  const onMouseMove = (e: any) => {
    if (!isDragging) return;
    handleInteraction(e.clientX, e.clientY);
  };

  const onMouseUp = () => {
    setIsDragging(false);
  };

  const onTouchStart = (e: any) => {
    setIsDragging(true);
    if (e.touches && e.touches.length > 0) {
      handleInteraction(e.touches[0].clientX, e.touches[0].clientY);
    }
  };

  const onTouchMove = (e: any) => {
    if (!isDragging) return;

    if (e.touches && e.touches.length > 0) {
      handleInteraction(e.touches[0].clientX, e.touches[0].clientY);
    }
  };

  useEffect(() => {
    if (!isDragging) return;

    if (typeof window === 'undefined') return;

    const handleWindowMouseMove = (e: any) => onMouseMove(e);
    const handleWindowMouseUp = () => onMouseUp();

    const handleWindowTouchMove = (e: any) => {
      if (e.touches && e.touches.length > 0) {
        handleInteraction(e.touches[0].clientX, e.touches[0].clientY);
      }
    };
    const handleWindowTouchEnd = () => onMouseUp();

    window.addEventListener('mousemove', handleWindowMouseMove);
    window.addEventListener('mouseup', handleWindowMouseUp);
    window.addEventListener('touchmove', handleWindowTouchMove);
    window.addEventListener('touchend', handleWindowTouchEnd);

    return () => {
      window.removeEventListener('mousemove', handleWindowMouseMove);
      window.removeEventListener('mouseup', handleWindowMouseUp);
      window.removeEventListener('touchmove', handleWindowTouchMove);
      window.removeEventListener('touchend', handleWindowTouchEnd);
    };
  }, [isDragging]);

  if (!isMounted) {
    return <Box sx={{ width: 360, height: 240, ...sx }} />;
  }

  const renderTicks = () => {
    const ticks = [];
    const step = 12.5;
    for (let v = -100; v <= 100; v += step) {
      const angle = valueToAngle(v);
      const isMajor = v % 50 === 0;
      const length = isMajor ? 18 : 10;
      const strokeWidth = isMajor ? 3 : 1.5;

      const x1 = CENTER_X + Math.cos(angle) * (HANDLE_RADIUS - 10);
      const y1 = CENTER_Y + Math.sin(angle) * (HANDLE_RADIUS - 10);
      const x2 = CENTER_X + Math.cos(angle) * (HANDLE_RADIUS - 10 - length);
      const y2 = CENTER_Y + Math.sin(angle) * (HANDLE_RADIUS - 10 - length);

      ticks.push(
        <line
          key={v}
          x1={x1} y1={y1}
          x2={x2} y2={y2}
          stroke="#ffffffff"
          strokeWidth={strokeWidth}
        />
      );

      if (Math.abs(v) === 100 || v === 0) {
        const textRadius = HANDLE_RADIUS - 45;
        const tx = CENTER_X + Math.cos(angle) * textRadius;
        const ty = CENTER_Y + Math.sin(angle) * textRadius;
        ticks.push(
          <text
            key={`t-${v}`}
            x={tx} y={ty}
            textAnchor="middle"
            dominantBaseline="middle"
            fill="#ffffffff"
            fontSize="14"
            fontWeight="bold"
            style={{ userSelect: 'none' }}
          >
            {v === -100 ? '左' : (v === 100 ? '右' : '0')}
          </text>
        );
      }
    }
    return ticks;
  };

  const currentAngle = valueToAngle(value);
  const handleX = CENTER_X + Math.cos(currentAngle) * (HANDLE_RADIUS - 5);
  const handleY = CENTER_Y + Math.sin(currentAngle) * (HANDLE_RADIUS - 5);

  return (
    <Box
      sx={{
        width: 360,
        height: 240,
        display: 'flex',
        justifyContent: 'center',
        overflow: 'hidden',
        userSelect: 'none',
        WebkitUserSelect: 'none',
        ...sx
      }}
    >
      <svg
        ref={svgRef}
        width={360}
        height={240}
        onMouseDown={onMouseDown}
        onTouchStart={onTouchStart}
        style={{
          cursor: isDragging ? 'grabbing' : 'grab',
          touchAction: 'none',
          display: 'block'
        }}
      >
        <defs>
          <linearGradient
            id="silverGradient1"
            gradientUnits="userSpaceOnUse"
            x1={CENTER_X - 14}
            y1={CENTER_Y + 14}
            x2={CENTER_X + 14}
            y2={CENTER_Y - 14}
          >
            <stop offset="0%" stopColor="#757575" />
            <stop offset="45%" stopColor="#9E9E9E" />
            <stop offset="70%" stopColor="#E8E8E8" />
            <stop offset="85%" stopColor="#9E9E9E" />
            <stop offset="90%" stopColor="#757575" />
            <stop offset="100%" stopColor="#757575" />
          </linearGradient>
          <linearGradient
            id="silverGradient2"
            gradientUnits="userSpaceOnUse"
            x1={CENTER_X - 60}
            y1={CENTER_Y + 60}
            x2={CENTER_X + 60}
            y2={CENTER_Y - 60}
          >
            <stop offset="100%" stopColor="#757575" />
            <stop offset="90%" stopColor="#9E9E9E" />
            <stop offset="85%" stopColor="#E8E8E8" />
            <stop offset="75%" stopColor="#9E9E9E" />
            <stop offset="45%" stopColor="#757575" />
            <stop offset="0%" stopColor="#757575" />
          </linearGradient>
          <linearGradient
            id="blackGradient"
            gradientUnits="userSpaceOnUse"
            x1={CENTER_X - 14}
            y1={CENTER_Y + 14}
            x2={CENTER_X + 14}
            y2={CENTER_Y - 14}
          >
            <stop offset="100%" stopColor="#151515" />
            <stop offset="90%" stopColor="#252525" />
            <stop offset="85%" stopColor="#353535" />
            <stop offset="70%" stopColor="#454545" />
            <stop offset="45%" stopColor="#555555" />
            <stop offset="0%" stopColor="#656565" />
          </linearGradient>
        </defs>

        <path
          d={`M ${CENTER_X - HANDLE_RADIUS},${CENTER_Y} A ${HANDLE_RADIUS},${HANDLE_RADIUS} 0 0,0 ${CENTER_X + HANDLE_RADIUS},${CENTER_Y}`}
          fill="none"
          stroke="url(#silverGradient1)"
          strokeWidth={STROKE_WIDTH}
          strokeLinecap="round"
        />

        {renderTicks()}

        {/* ハンドルバー */}
        <line
          x1={CENTER_X}
          y1={CENTER_Y}
          x2={handleX}
          y2={handleY}
          stroke="url(#silverGradient1)"
          strokeWidth={10}
          strokeLinecap="round"
        />

        {/* 中心軸 */}
        <circle cx={CENTER_X} cy={CENTER_Y} r={14} fill="url(#silverGradient2)" />

        {/* つまみ */}
        <circle
          cx={handleX}
          cy={handleY}
          r={12}
          fill="url(#silverGradient1)"
          stroke="#FFFFFF"
          strokeWidth={0.5}
        />

        {/* 現在値表示 */}
        <text
          x={CENTER_X}
          y={CENTER_Y + HANDLE_RADIUS + 40}
          textAnchor="middle"
          fill="#ffffffff"
          fontSize="24"
          fontWeight="bold"
          style={{ userSelect: 'none' }}
        >
          {value}
        </text>
      </svg>
    </Box >
  );
};

export const MechanicalSwitch = styled(Switch)(({ theme }) => ({
  width: 68,
  height: 34,
  padding: 0,
  '& .MuiSwitch-switchBase': {
    padding: 0,
    margin: 4,
    transitionDuration: '200ms',
    borderRadius: 0,
    '&.Mui-checked': {
      transform: 'translateX(34px)',
      '& .MuiSwitch-thumb': {
        backgroundColor: '#424242',
        border: '1px solid #212121',
      },
      '& + .MuiSwitch-track': {
        backgroundColor: '#9e9e9e',
        opacity: 1,
        border: '1px solid #757575',
      },
    },
  },
  '& .MuiSwitch-thumb': {
    boxSizing: 'border-box',
    width: 26,
    height: 26,
    borderRadius: 0,
    backgroundColor: '#e0e0e0',
    border: '1px solid #9e9e9e',
    boxShadow: 'none',
  },
  '& .MuiSwitch-track': {
    borderRadius: 0,
    backgroundColor: '#cfcfcf',
    opacity: 1,
    border: '1px solid #9e9e9e',
    transition: theme.transitions.create(['background-color'], {
      duration: 200,
    }),
  },
}));


function useLocalState(arg0: boolean): [any, any] {
  throw new Error('Function not implemented.');
}
