import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import {useThemeConfig} from '@docusaurus/theme-common';
import {useLocation} from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

function Navbar() {
  const themeConfig = useThemeConfig();
  const location = useLocation();
  const {navbar: {title, items = []}} = themeConfig;

  return (
    <nav
      className={clsx('navbar', 'navbar--fixed-top', styles.navbar)}
      role="navigation"
      aria-label="Main navigation">
      <div className="container">
        <div className={styles.navbarBrand}>
          <Link className="navbar__brand" to={useBaseUrl('/')}>
            <div className={styles.logoContainer}>
              <div className={styles.roboticsLogo}></div>
              <strong className={styles.navbarTitle}>{title}</strong>
            </div>
          </Link>
        </div>

        {items?.length > 0 && (
          <ul className="navbar__items">
            {items.map((item, i) => (
              <li className="navbar__item" key={i}>
                <Link
                  activeClassName="navbar__link--active"
                  className="navbar__link"
                  to={item.to}
                  target={item.target}>
                  {item.label}
                </Link>
              </li>
            ))}
          </ul>
        )}
      </div>
    </nav>
  );
}

export default Navbar;