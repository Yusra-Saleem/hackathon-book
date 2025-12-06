/**
 * Minimal Docusaurus v3 config for my-ai-book
 * Recreated to replace the corrupted file.
 */
module.exports = {
  title: 'Physical AI ',
  tagline: 'Physical AI & Humanoid Robots',
  url: 'http://localhost',
  baseUrl: '/',
  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
        },
        blog: {
          showReadingTime: true,
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],
  themeConfig: {
    navbar: {
      title: 'Physical AI & Humanoid Robots',
      items: [
        { to: '/docs/Introduction/Foundations-Hardware', label: 'TextBook', position: 'left' },
        { href: 'https://github.com', label: 'GitHub', position: 'right' },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            { label: 'Introduction', to: '/docs/Introduction' },
          ],
        },
        {
          title: 'Community',
          items: [
            { label: 'GitHub', href: 'https://github.com' },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} My AI Book`,
    },
  },
};
