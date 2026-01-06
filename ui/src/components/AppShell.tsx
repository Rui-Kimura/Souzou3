"use client";

import React, { useState } from 'react';
import { 
  AppBar, Box, Drawer, IconButton, List, ListItem, 
  ListItemButton, ListItemText, Toolbar, Typography, Divider 
} from "@mui/material";
import { styled } from '@mui/material/styles';
import MenuIcon from '@mui/icons-material/Menu';
import { useRouter, usePathname } from 'next/navigation';
import { MENU_ITEMS, SITE_NAME } from '@/constants'; 

const DrawerHeader = styled('div')(({ theme }) => ({
  display: 'flex',
  alignItems: 'center',
  padding: theme.spacing(0, 1),
  ...theme.mixins.toolbar,
  justifyContent: 'flex-end',
}));

export default function AppShell({ children }: { children: React.ReactNode }) {
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
      {isTopPage && <DrawerHeader />} 
      {!isTopPage && (
        <>
          <Box sx={{ py: 2 }}>
             <Typography variant="h6">{SITE_NAME}</Typography>
          </Box>
          <Divider />
        </>
      )}
      <List sx={{ pt: isTopPage ? 0 : 0 }}>
        {MENU_ITEMS.map((item) => (
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
    <Box sx={{ display: 'flex', height: '100dvh' }}>
      {!isControllerPage && (
        <AppBar 
          component="nav" 
          position="fixed" 
          color="primary" 
          sx={{ zIndex: (theme) => theme.zIndex.drawer + 1 }}
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
              {SITE_NAME}
            </Typography>
            {!isTopPage && <Box sx={{ width: 48 }} />} 
          </Toolbar>
        </AppBar>
      )}

      <Box
        component="nav"
        sx={{ width: isTopPage ? drawerWidth : 0, flexShrink: { sm: 0 } }}
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

      <Box 
        component="main" 
        sx={{ 
          flexGrow: 1, 
          width: isTopPage ? { 
            xs: `calc(100% - ${drawerWidth.xs}px)`, 
            sm: `calc(100% - ${drawerWidth.sm}px)`, 
            md: `calc(100% - ${drawerWidth.md}px)` 
          } : "100%",
          height: '100%',
          display: 'flex',
          flexDirection: 'column',
          overflow: 'hidden'
        }}
      >
        {!isControllerPage && <DrawerHeader />}
        <Box sx={{ 
          flexGrow: 1, 
          overflowY: 'auto', 
          overflowX: 'hidden', 
          position: 'relative'
        }}>
          {children}
        </Box>
      </Box>
    </Box>
  );
}