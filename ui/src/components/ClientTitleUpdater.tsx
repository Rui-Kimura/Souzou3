'use client';

import { useEffect } from 'react';
import { usePathname } from 'next/navigation';
import { MENU_ITEMS, SITE_NAME } from '@/constants';

export default function ClientTitleUpdater() {
  const pathname = usePathname();

  useEffect(() => {
    const currentItem = MENU_ITEMS.find((item) => item.url === pathname);
    
    const title = currentItem ? SITE_NAME + " - " + currentItem.label : SITE_NAME;

    document.title = title;
    
  }, [pathname]);

  return null; 
}