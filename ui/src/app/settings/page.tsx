'use client';

import React, { useState, useEffect } from 'react';

import {
  Box,
  Container,
  Typography,
  Paper,
  List,
  ListItem,
  ListItemText,
  ListItemIcon,
  IconButton,
  Divider,
  CircularProgress,
  Tooltip,
  Snackbar,
  Alert,
} from '@mui/material';

import {
  Wifi as WifiIcon,
  Info as InfoIcon,
  ContentCopy as CopyIcon,
  Refresh as RefreshIcon,
  Settings as SettingsIcon,
} from '@mui/icons-material';

interface SettingItem {
  id: string;
  label: string;
  value?: string;
  icon: React.ReactNode;
  action?: React.ReactNode;
  loading?: boolean;
}

interface SettingSection {
  title: string;
  items: SettingItem[];
}

export default function SettingsPage() {
  const [ipAddress, setIpAddress] = useState<string>('---.---.---.---');
  const [loadingIp, setLoadingIp] = useState<boolean>(false);
  const [snackbarOpen, setSnackbarOpen] = useState(false);

  const fetchIpAddress = async () => {
    setLoadingIp(true);
    try {
      const res = await fetch('/api/local-ip');
      const data = await res.json();
      setIpAddress(data.ip);
    } catch (error) {
      console.error('IP fetch failed', error);
      setIpAddress('取得失敗');
    } finally {
      setLoadingIp(false);
    }
  };

  useEffect(() => {
    fetchIpAddress();
  }, []);

  const handleCopy = (text: string) => {
    navigator.clipboard.writeText(text);
    setSnackbarOpen(true);
  };

  const settingsConfig: SettingSection[] = [
    {
      title: 'ネットワーク情報',
      items: [
        {
          id: 'ip-address',
          label: 'IP アドレス',
          value: ipAddress,
          icon: <WifiIcon />,
          loading: loadingIp,
          action: (
            <Box>
               <Tooltip title="再取得">
                <IconButton edge="end" onClick={fetchIpAddress} sx={{ mr: 1 }}>
                  <RefreshIcon />
                </IconButton>
              </Tooltip>
              <Tooltip title="コピー">
                <IconButton edge="end" onClick={() => handleCopy(ipAddress)}>
                  <CopyIcon />
                </IconButton>
              </Tooltip>
            </Box>
          ),
        },
      ],
    },
    {
      title: 'アプリケーション情報',
      items: [
        {
          id: 'version',
          label: 'バージョン',
          value: 'v0.114.514.810.1919',
          icon: <InfoIcon />,
        },
      ],
    },
  ];

  return (
    <Box sx={{ minHeight: '100vh', bgcolor: 'background.default', py: 4 }}>
      <Container maxWidth="sm">
        <Box sx={{ mb: 4, display: 'flex', alignItems: 'center', gap: 2 }}>
          <SettingsIcon sx={{ fontSize: 40, color: 'primary.main' }} />
          <Typography variant="h4" component="h1" sx={{ fontWeight: 'bold', color: '#333' }}>
            設定
          </Typography>
        </Box>

        {settingsConfig.map((section) => (
          <Box key={section.title} sx={{ mb: 3 }}>
            <Typography
              variant="subtitle1"
              sx={{ mb: 1, ml: 1, fontWeight: 'bold', color: 'text.secondary' }}
            >
              {section.title}
            </Typography>

            <Paper elevation={0} sx={{ border: '1px solid', borderColor: 'divider', overflow: 'hidden' }}>
              <List disablePadding>
                {section.items.map((item, itemIndex) => (
                  <React.Fragment key={item.id}>
                    {itemIndex > 0 && <Divider component="li" />}
                    <ListItem 
                      sx={{ py: 2 }}
                      secondaryAction={item.action}
                    >
                      <ListItemIcon sx={{ color: 'primary.main' }}>
                        {item.icon}
                      </ListItemIcon>
                      
                      <ListItemText
                        primary={item.label}
                        secondary={
                          item.loading ? (
                            <Box component="span" sx={{ display: 'flex', alignItems: 'center', mt: 0.5 }}>
                               <CircularProgress size={16} sx={{ mr: 1, color: 'primary.main' }} />
                               <Typography component="span" variant="caption">取得中...</Typography>
                            </Box>
                          ) : (
                            <Typography component="span" variant="body1" sx={{ fontWeight: 500, mt: 0.5, display: 'block' }}>
                              {item.value}
                            </Typography>
                          )
                        }
                      />
                    </ListItem>
                  </React.Fragment>
                ))}
              </List>
            </Paper>
          </Box>
        ))}
      </Container>

      <Snackbar
        open={snackbarOpen}
        autoHideDuration={2000}
        onClose={() => setSnackbarOpen(false)}
        anchorOrigin={{ vertical: 'bottom', horizontal: 'center' }}
      >
        <Alert severity="success" sx={{ width: '100%' }}>
          クリップボードにコピーしました
        </Alert>
      </Snackbar>
    </Box>
  );
}