import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import {useThemeConfig} from '@docusaurus/theme-common';
import styles from './styles.module.css';

function Footer() {
  const {footer} = useThemeConfig();
  const {copyright, links = [], style} = footer || {};

  return (
    <footer
      className={clsx(
        'footer',
        {
          'footer--primary': style === 'primary',
          'footer--dark': style === 'dark',
        },
        styles.footer,
      )}>
      <div className="container container-fluid">
        <div className={styles.footerContent}>
          <div className={styles.footerSection}>
            <div className={styles.footerLogo}>
              <div className={styles.roboticsLogo}></div>
              <h3 className={styles.footerTitle}>Physical AI & Robotics</h3>
            </div>
            <p className={styles.footerDescription}>
              Advanced simulation techniques for AI and robotics education.
            </p>
          </div>

          {links?.length > 0 && (
            <div className={styles.footerSections}>
              {links.map((section, i) => (
                <div className={styles.footerSection} key={i}>
                  <h4 className={styles.footerSectionTitle}>
                    {section.title}
                  </h4>
                  <ul className={styles.footerLinks}>
                    {section.items?.map((item, j) => (
                      <li key={j} className={styles.footerLinkItem}>
                        <Link
                          to={item.to}
                          className={styles.footerLink}
                          target={item.target}>
                          {item.label}
                        </Link>
                      </li>
                    ))}
                  </ul>
                </div>
              ))}
            </div>
          )}
        </div>

        {copyright && (
          <div className={styles.copyright}>
            <div className={styles.copyrightText}>{copyright}</div>
          </div>
        )}
      </div>
    </footer>
  );
}

export default Footer;