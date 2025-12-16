"use client";

import React, { useState } from 'react';
import { 
  AppBar, 
  Box, 
  CssBaseline, 
  Drawer, 
  IconButton, 
  List, 
  ListItem, 
  ListItemButton, 
  ListItemText, 
  Toolbar, 
  Typography, 
  Divider 
} from "@mui/material";
import { styled } from '@mui/material/styles'; // 追加
import MenuIcon from '@mui/icons-material/Menu';
import { useRouter, usePathname } from 'next/navigation';

// --- 追加: ヘッダーの高さ分を確保するための正確なスペーサー ---
const DrawerHeader = styled('div')(({ theme }) => ({
  display: 'flex',
  alignItems: 'center',
  padding: theme.spacing(0, 1),
  // theme.mixins.toolbar のスタイル（高さなど）を継承します
  ...theme.mixins.toolbar,
  justifyContent: 'flex-end',
}));
// -------------------------------------------------------

const menuItems = [
  { label: "ホーム", url: "/" },
  { label: "マップ表示", url: "/map" },
  { label: "自動移動", url: "/automation" },
  { label: "手動操作", url: "/controller" },
  { label: "設定", url: "/settings" },
];

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  const router = useRouter();
  const pathname = usePathname();
  const [mobileOpen, setMobileOpen] = useState(false);

  const isTopPage = pathname === "/";
  const isControllerPage = pathname === "/controller";

  const drawerWidth = { xs: 140, sm: 200, md: 240 };

  const handleDrawerToggle = () => {
    setMobileOpen(!mobileOpen);
  };

  const handleNavigation = (url: string) => {
    router.push(url);
    setMobileOpen(false);
  };

  const drawerContent = (
    <Box sx={{ textAlign: 'center', height: "100%" }}>
      {/* メニュー側もDrawerHeaderを使って高さを合わせる */}
      {isTopPage && <DrawerHeader />} 
      
      {!isTopPage && (
        <>
          <Box sx={{ py: 2 }}>
             <Typography variant="h6">MOFEL</Typography>
          </Box>
          <Divider />
        </>
      )}

      <List sx={{ pt: isTopPage ? 0 : 0 }}>
        {menuItems.map((item) => (
          <ListItem key={item.label} disablePadding>
            <ListItemButton onClick={() => handleNavigation(item.url)} sx={{ py: 1.5 }}>
              <ListItemText 
                primary={item.label} 
                primaryTypographyProps={{ 
                  fontSize: isTopPage ? { xs: "0.8rem", sm: "1rem" } : "1rem",
                  noWrap: true 
                }}
              />
            </ListItemButton>
          </ListItem>
        ))}
      </List>
    </Box>
  );

  return (
    <html lang="ja">
      <body>
        <CssBaseline />
        
        <Box sx={{ display: 'flex', height: '100dvh' }}>
          
          {/* ヘッダー */}
          {!isControllerPage && (
            <AppBar 
              component="nav" 
              position="fixed" 
              sx={{ 
                bgcolor: "#4a90e2",
                // drawerより上に表示する設定
                zIndex: (theme) => theme.zIndex.drawer + 1 
              }}
            >
              <Toolbar>
                {!isTopPage && (
                  <IconButton
                    color="inherit"
                    aria-label="open drawer"
                    edge="start"
                    onClick={handleDrawerToggle}
                    sx={{ mr: 2 }}
                  >
                    <MenuIcon />
                  </IconButton>
                )}
                
                <Typography
                  variant="h6"
                  component="div"
                  sx={{ flexGrow: 1, textAlign: 'center', fontWeight: 'bold' }}
                >
                  MOFEL
                </Typography>
                
                {!isTopPage && <Box sx={{ width: 48 }} />} 
              </Toolbar>
            </AppBar>
          )}

          {/* メニュー */}
          <Box
            component="nav"
            sx={{ 
              width: isTopPage ? drawerWidth : 0, 
              flexShrink: { sm: 0 } 
            }}
          >
            <Drawer
              variant={isTopPage ? "permanent" : "temporary"}
              open={isTopPage ? true : mobileOpen}
              onClose={handleDrawerToggle}
              ModalProps={{ keepMounted: true }}
              sx={{
                '& .MuiDrawer-paper': { 
                  boxSizing: 'border-box', 
                  width: isTopPage ? drawerWidth : 240,
                  borderRight: isTopPage ? "1px solid #e0e0e0" : undefined,
                },
              }}
            >
              {drawerContent}
            </Drawer>
          </Box>

          {/* メインコンテンツ */}
          <Box 
            component="main" 
            sx={{ 
              flexGrow: 1, 
              width: isTopPage ? { 
                xs: `calc(100% - ${drawerWidth.xs}px)`, 
                sm: `calc(100% - ${drawerWidth.sm}px)`,
                md: `calc(100% - ${drawerWidth.md}px)` 
              } : "100%",
              height: '100%', // 画面いっぱいに広げる
              display: 'flex',
              flexDirection: 'column',
              overflow: 'hidden' // ここではスクロールさせず、内側のBoxでさせる
            }}
          >
            {/* ここにスペーサーを配置 
               DrawerHeaderを使うことで、スマホ/PCごとのヘッダー高さに正確に追従します
            */}
            {!isControllerPage && <DrawerHeader />}
            
            {/* 実際のコンテンツエリア */}
            <Box sx={{ 
              flexGrow: 1, 
              overflowY: 'auto', // コンテンツが多い場合のみここでスクロール
              overflowX: 'hidden',
              position: 'relative'
            }}>
              {children}
            </Box>
          </Box>
        </Box>
      </body>
    </html>
  );
}